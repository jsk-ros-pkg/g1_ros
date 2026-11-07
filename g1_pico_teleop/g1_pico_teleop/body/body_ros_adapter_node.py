"""Workstation bridge: private PICO ZMQ stream -> existing ROS SONIC services.

Never binds a body-command port or modifies the robot's deployment. The node
must be armed while the manager is OFF. Supported manager modes are OFF,
PLANNER (idle balance), and POSE (SMPL following). Disarm returns to balance;
it does not issue the manager's damping/stop command.
"""

import math
import signal
import time
from urllib.parse import urlsplit

from gear_sonic_interfaces.msg import SmplMotion
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger

from .manager_receiver import ManagerReceiver
from .manager_wire import ManagerWireError, decode_manager_message


def _positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be positive and finite")
    return value


def validate_endpoint(endpoint):
    parsed = urlsplit(endpoint)
    if (parsed.scheme != "tcp" or not parsed.hostname or parsed.path
            or parsed.query or parsed.fragment or parsed.username or parsed.password
            or parsed.port is None or parsed.port in (5556, 5557, 5560)):
        raise ValueError("body_zmq_endpoint must use a private TCP port, e.g. 5558; "
                         "5556/5557/5560 are reserved for SONIC and optical hands")
    return endpoint


def to_smpl_message(window, index, stamp):
    msg = SmplMotion()
    msg.header.stamp = stamp
    msg.frame_index = int(window.frame_index[index])
    msg.smpl_pose = window.smpl_pose[index].reshape(-1).astype(float).tolist()
    msg.smpl_joints = window.smpl_joints[index].reshape(-1).astype(float).tolist()
    q = window.body_quat_w[index]
    msg.body_quat.w, msg.body_quat.x, msg.body_quat.y, msg.body_quat.z = map(float, q)
    msg.joint_pos = window.joint_pos[index].astype(float).tolist()
    msg.joint_vel = window.joint_vel[index].astype(float).tolist()
    return msg


class PicoBodyAdapter(Node):
    def __init__(self):
        super().__init__("pico_body_adapter")
        endpoint = validate_endpoint(str(self.declare_parameter(
            "body_zmq_endpoint", "tcp://10.42.0.1:5558").value))
        prefix = str(self.declare_parameter("sonic_prefix", "/gear_sonic_interface").value).rstrip("/")
        self._source_timeout = _positive(self.declare_parameter("source_timeout", 0.5).value, "source_timeout")
        self._service_timeout = _positive(self.declare_parameter("service_timeout", 3.0).value, "service_timeout")
        self._settle_time = _positive(self.declare_parameter("balance_duration", 3.0).value, "balance_duration")
        self._pose_start_timeout = _positive(self.declare_parameter("pose_start_timeout", 5.0).value, "pose_start_timeout")

        self._publisher = self.create_publisher(SmplMotion, f"{prefix}/smpl_motion", 64)
        self._zero_velocity = self.create_publisher(Twist, "/cmd_vel", 1)
        self._height = self.create_publisher(Float64, f"{prefix}/target_height", 1)
        self._sonic_clients = {
            "mode": self.create_client(Trigger, f"{prefix}/mode/default"),
            "control": self.create_client(SetBool, f"{prefix}/enable_control"),
            "stream": self.create_client(SetBool, f"{prefix}/enable_smpl_stream"),
        }
        self._receiver = ManagerReceiver(endpoint)

        self._armed = False
        self._mode = None
        self._state_at = 0.0
        self._mode_at = 0.0
        self._window = None
        self._pose_at = 0.0
        self._input_index = -1
        self._forwarded_index = -1
        self._source_stamp = None
        # Ownership flags include requests in flight, whose effects may be delayed.
        self._control_owned = False
        self._stream_maybe_enabled = False
        self._mode_reset = False
        self._balance_since = 0.0
        self._recovering = False
        self._rpc = None
        self._retry_at = 0.0
        self._last_log = {}
        self.create_service(SetBool, "~/enable", self._on_enable)
        self._timer = self.create_timer(0.02, self._on_timer)
        self.get_logger().info(f"Receiving manager on {endpoint}; body output disabled. "
                               "Arm while manager is OFF; stop returns to balance.")

    def _warning(self, key, message):
        now = time.monotonic()
        if now - self._last_log.get(key, 0.0) >= 2.0:
            self.get_logger().warning(message)
            self._last_log[key] = now

    def _clear_pose(self):
        self._window = None
        self._pose_at = 0.0
        self._input_index = -1
        self._forwarded_index = -1
        self._source_stamp = None

    def _on_enable(self, request, response):
        if not request.data:
            self.request_stop("operator request")
            response.success = True
            response.message = "body disarmed; returning to balance" if self._recovering else "body already disabled"
            return response
        now = time.monotonic()
        missing = [name for name, client in self._sonic_clients.items() if not client.service_is_ready()]
        if self._recovering or self._rpc is not None:
            reason = "wait for pending service calls and return to balance"
        elif self._armed:
            response.success, response.message = True, "body already armed"
            return response
        elif self._mode != 0 or now - self._state_at > self._source_timeout:
            reason = "need fresh manager_state in OFF mode on the private body port"
        elif missing:
            reason = "ROS SONIC services unavailable: " + ", ".join(missing)
        else:
            self._clear_pose()
            self._mode_reset = False
            self._armed = True
            response.success = True
            response.message = "body armed; use A+B+X+Y for balance, then A+X for POSE"
            self.get_logger().info(response.message)
            return response
        response.success, response.message = False, reason
        return response

    def request_stop(self, reason):
        was_active = self._armed or self._control_owned or self._stream_maybe_enabled
        self._armed = False
        self._clear_pose()
        self._recovering = self._control_owned or self._stream_maybe_enabled
        if was_active:
            self._warning("stop", f"Body disarmed: {reason}; requesting return to balance")

    def _consume(self, message, now):
        topic, value = decode_manager_message(message)
        if topic == b"command":
            if value:
                self.request_stop("manager stop button")
            return
        if topic == b"manager_state":
            previous_mode = self._mode
            if value != self._mode:
                if self._mode == 1:
                    self._clear_pose()  # manager resets frame_index on leaving POSE
                self._mode_at = now
                self.get_logger().info(f"Manager mode: {self._mode} -> {value}")
            self._mode = value
            self._state_at = now
            if self._armed and value not in (0, 1, 2):
                self.request_stop("unsupported manager mode; use OFF/PLANNER/POSE only")
            if value == 0 and self._armed and (previous_mode not in (None, 0) or self._control_owned):
                self.request_stop("manager returned to OFF or restarted")
            return
        newest = int(value.frame_index[-1])
        if newest < self._input_index:
            self.request_stop("pose frame counter went backwards; restart manager before rearming")
            return
        if value.source_stamp is not None and self._source_stamp is not None:
            if value.source_stamp < self._source_stamp:
                self.request_stop("pose timestamp went backwards")
                return
            if value.source_stamp == self._source_stamp:
                return
        if newest == self._input_index:
            return  # duplicates do not refresh the watchdog
        self._input_index = newest
        self._source_stamp = value.source_stamp
        self._window, self._pose_at = value, now

    def _receive(self, now):
        messages, fault = self._receiver.take()
        if fault:
            self.request_stop(fault)
        for message, received_at in messages:
            if now - received_at > self._source_timeout:
                self.request_stop("queued manager message is stale")
                continue
            try:
                self._consume(message, received_at)
            except ManagerWireError as exc:
                self.request_stop(f"invalid manager message: {exc}")
                self._warning("wire", str(exc))

    def _call(self, kind, value, now):
        client = self._sonic_clients[kind]
        if now < self._retry_at:
            return
        if not client.service_is_ready():
            self.request_stop(f"{kind} service unavailable")
            self._warning("service", "Waiting for ROS services to confirm return to balance")
            self._retry_at = now + 0.5
            return
        request = Trigger.Request() if kind == "mode" else SetBool.Request(data=value)
        try:
            future = client.call_async(request)
        except Exception as exc:
            self.request_stop(f"{kind} request failed: {exc}")
            self._retry_at = now + 0.5
            return
        if kind == "control" and value:
            self._control_owned = True
        if kind == "stream" and value:
            self._stream_maybe_enabled = True
        self._rpc = [kind, value, future, now, False]

    def _poll_call(self, now):
        if self._rpc is None:
            return
        kind, value, future, started, timed_out = self._rpc
        if not future.done():
            if not timed_out and now - started > self._service_timeout:
                self._rpc[4] = True
                self.request_stop(f"{kind} service timed out")
            # Keep the outstanding request. A late enable must finish BEFORE
            # compensating disable calls, and it must never restart forwarding.
            return
        self._rpc = None
        try:
            result = future.result()
            if result is None or not result.success:
                raise RuntimeError("empty response" if result is None else result.message)
        except Exception as exc:
            self.request_stop(f"{kind} service rejected/failed: {exc}")
            self._retry_at = now + 0.5
            return
        if kind == "mode":
            self._mode_reset = True
        elif kind == "control":
            self._control_owned = value
            if value:
                self._balance_since = now
                self.get_logger().info("SONIC balancing; wait before entering POSE")
        elif kind == "stream":
            self._stream_maybe_enabled = value
            if value:
                self.get_logger().info("Body following active through ROS SmplMotion")
            else:
                self._balance_since = now
        if not self._armed:
            self._recovering = self._control_owned or self._stream_maybe_enabled
            if not self._recovering:
                self.get_logger().info("Body disabled; ROS bridge returned to balance (policy still running)")

    def _on_timer(self):
        now = time.monotonic()
        if self._armed and now - self._state_at > self._source_timeout:
            self.request_stop("manager heartbeat stale")
        if (self._armed and self._mode == 1 and self._stream_maybe_enabled
                and now - self._pose_at > self._source_timeout):
            self.request_stop("pose stream stale")
        self._receive(now)
        self._poll_call(now)
        if self._rpc is not None:
            return
        if self._recovering:
            if self._stream_maybe_enabled:
                self._call("stream", False, now)
            elif self._control_owned:
                self._call("control", False, now)
            else:
                self._recovering = False
            return
        if not self._armed or self._mode == 0:
            return
        if any(not client.service_is_ready() for client in self._sonic_clients.values()):
            self.request_stop("ROS SONIC service disappeared")
            return
        if self._mode == 2:
            if self._stream_maybe_enabled:
                self._call("stream", False, now)
            elif not self._control_owned:
                self._zero_velocity.publish(Twist())
                self._height.publish(Float64(data=-1.0))
                self._call("control" if self._mode_reset else "mode", True, now)
            return
        if not self._control_owned:
            self.request_stop("enter PLANNER and wait for balance before POSE")
            return
        fresh_pose = self._window is not None and now - self._pose_at <= self._source_timeout
        if not fresh_pose:
            if now - self._mode_at > self._pose_start_timeout:
                self.request_stop("no fresh pose after entering POSE")
            return
        if not self._stream_maybe_enabled:
            if now - self._balance_since >= self._settle_time:
                self._call("stream", True, now)
            return
        stamp = self.get_clock().now().to_msg()
        for index, frame_id in enumerate(self._window.frame_index):
            if int(frame_id) > self._forwarded_index:
                self._publisher.publish(to_smpl_message(self._window, index, stamp))
                self._forwarded_index = int(frame_id)

    def close(self):
        self._timer.cancel()
        self._receiver.close()
        self.destroy_node()


def main(args=None):
    # Keep ROS alive during Ctrl+C so the balancing service requests can finish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt
    old_int = signal.signal(signal.SIGINT, interrupt)
    old_term = signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = PicoBodyAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch may relay the terminal's SIGINT again. Preserve the
        # bounded service cleanup even when that second interrupt arrives.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.request_stop("node shutdown")
            deadline = time.monotonic() + 2 * node._service_timeout + 1.0
            try:
                while rclpy.ok() and (node._recovering or node._rpc is not None) and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)
            except KeyboardInterrupt:
                pass
            if node._recovering or node._rpc is not None:
                node.get_logger().error("Could not confirm return to balance; check the robot before proceeding")
            node.close()
        signal.signal(signal.SIGTERM, old_term)
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, old_int)


if __name__ == "__main__":
    main()
