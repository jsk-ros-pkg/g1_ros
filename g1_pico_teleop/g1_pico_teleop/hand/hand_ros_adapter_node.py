"""Hand-only ROS 2 adapter for PICO OpenXR optical hand tracking.

This node intentionally has no Gear Sonic body imports, publishers, services,
or state. Body control is owned by the robot's official manager and SONIC.
The body-sized field in the compatible UDP v1 envelope is ignored.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import socket
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .hand_command import HandCommandError, joint_names, make_jtc_command
from .optical import (
    NATIVE6_LOWER,
    NATIVE6_ORDER,
    NATIVE6_UPPER,
    OpticalCalibration,
    OpticalRetargetingError,
)
from ..wire import (
    DATAGRAM_SIZE, OPTICAL_MESSAGE_SIZE, OPTICAL_TOPIC,
    PicoUdpFrame, WireError, decode_frame, decode_optical_message,
)


_HAND_TOPICS = {
    "left": "/left_hand_controller/joint_trajectory",
    "right": "/right_hand_controller/joint_trajectory",
}


def _positive(name: str, value: Any) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be positive and finite")
    return result


def _port(value: Any) -> int:
    result = int(value)
    if not 1 <= result <= 65535:
        raise ValueError("listen_port must be in 1..65535")
    return result


def _duration(seconds: float) -> Duration:
    whole = int(seconds)
    nanoseconds = int(round((seconds - whole) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanoseconds)


def _default_calibration_path() -> str:
    try:
        share = Path(get_package_share_directory("g1_pico_teleop"))
    except Exception:
        share = Path(__file__).resolve().parents[2]
    return str(share / "config" / "pico_native6_calibration.json")


def _load_calibrations(path: str) -> tuple[OpticalCalibration, OpticalCalibration]:
    source = Path(path).expanduser().resolve()
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise OpticalRetargetingError(f"cannot load calibration {source}: {exc}") from exc
    if payload.get("schema") != "inspire.optical.calibration.bundle.v1":
        raise OpticalRetargetingError("unknown optical calibration bundle schema")
    if tuple(payload.get("joint_order", ())) != NATIVE6_ORDER:
        raise OpticalRetargetingError("optical calibration joint_order is not native6")

    calibrations = []
    for side in ("left", "right"):
        hand = payload.get(side)
        if not isinstance(hand, dict) or hand.get("side") != side:
            raise OpticalRetargetingError(f"calibration is missing the {side} hand")
        if hand.get("schema") != "inspire.optical.calibration.v1":
            raise OpticalRetargetingError(f"unknown {side} calibration schema")
        if tuple(hand.get("joint_order", ())) != NATIVE6_ORDER:
            raise OpticalRetargetingError(f"{side} calibration joint_order is not native6")
        calibrations.append(
            OpticalCalibration(
                side=side,
                open_features=hand.get("open_features"),
                closed_features=hand.get("closed_features"),
                open_q=hand.get("open_q"),
                closed_q=hand.get("closed_q"),
            )
        )
    return calibrations[0], calibrations[1]


class PicoHandAdapter(Node):
    """Publish only Inspire hand trajectories from paired optical hand frames."""

    def __init__(self) -> None:
        super().__init__("pico_hand_adapter")
        self.declare_parameter("listen_address", "127.0.0.1")
        self.declare_parameter("listen_port", 5570)
        self.declare_parameter("hand_zmq_endpoint", "")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("source_timeout", 0.5)
        self.declare_parameter("joint_state_timeout", 0.5)
        self.declare_parameter("trajectory_duration", 0.04)
        self.declare_parameter("calibration_file", _default_calibration_path())
        self.declare_parameter("require_hand_state", True)
        self.declare_parameter("velocity_margin", 0.8)

        self._listen_address = str(self.get_parameter("listen_address").value)
        self._listen_port = _port(self.get_parameter("listen_port").value)
        self._publish_rate = _positive(
            "publish_rate", self.get_parameter("publish_rate").value
        )
        self._source_timeout = _positive(
            "source_timeout", self.get_parameter("source_timeout").value
        )
        self._joint_state_timeout = _positive(
            "joint_state_timeout", self.get_parameter("joint_state_timeout").value
        )
        self._trajectory_duration = _positive(
            "trajectory_duration", self.get_parameter("trajectory_duration").value
        )
        self._require_hand_state = bool(self.get_parameter("require_hand_state").value)
        self._velocity_margin = float(self.get_parameter("velocity_margin").value)
        if not math.isfinite(self._velocity_margin) or not 0.0 < self._velocity_margin <= 1.0:
            raise ValueError("velocity_margin must be in (0, 1]")

        calibration_file = str(self.get_parameter("calibration_file").value).strip()
        if not calibration_file:
            calibration_file = _default_calibration_path()
        self._left_calibration, self._right_calibration = _load_calibrations(
            calibration_file
        )

        self._hand_zmq_endpoint = str(self.get_parameter("hand_zmq_endpoint").value).strip()
        self._zmq_context = None
        self._zmq = None
        self._source_session = None
        if self._hand_zmq_endpoint:
            import zmq
            self._zmq = zmq
            self._zmq_context = zmq.Context()
            self._socket = self._zmq_context.socket(zmq.SUB)
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.setsockopt(zmq.RCVHWM, 1)
            self._socket.setsockopt(zmq.CONFLATE, 1)
            self._socket.setsockopt(zmq.MAXMSGSIZE, OPTICAL_MESSAGE_SIZE)
            self._socket.setsockopt(zmq.SUBSCRIBE, OPTICAL_TOPIC)
            self._socket.connect(self._hand_zmq_endpoint)
            input_description = f"ZMQ {self._hand_zmq_endpoint}"
        else:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, DATAGRAM_SIZE * 16)
            self._socket.setblocking(False)
            self._socket.bind((self._listen_address, self._listen_port))
            input_description = f"udp://{self._listen_address}:{self._listen_port}"

        self._latest_frame: PicoUdpFrame | None = None
        self._latest_received_at = 0.0
        self._last_sequence = -1
        self._last_device_timestamp_ns = -1
        self._last_source_timestamp_ns = -1
        self._joint_positions: dict[str, float] = {}
        self._joint_state_received_at = 0.0
        self._previous_hand_q: dict[str, np.ndarray | None] = {
            "left": None,
            "right": None,
        }
        self._last_hand_command_at = 0.0
        self._requested_enabled = False
        self._outputs_enabled = False
        self._last_log: dict[str, float] = {}
        self._invalid_datagrams = 0
        self._out_of_order_datagrams = 0

        self._hand_publishers = {
            side: self.create_publisher(JointTrajectory, topic, 1)
            for side, topic in _HAND_TOPICS.items()
        }
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_service(SetBool, "~/enable", self._on_enable)
        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)
        self.get_logger().info(
            f"Hand-only adapter receiving {input_description} "
            f"at {self._publish_rate:.1f} Hz; output is disabled; "
            f"calibration={calibration_file}"
        )

    def _log_throttled(
        self, level: str, key: str, message: str, period: float = 2.0
    ) -> None:
        now = time.monotonic()
        if now - self._last_log.get(key, 0.0) < period:
            return
        self._last_log[key] = now
        getattr(self.get_logger(), level)(message)

    def _on_joint_state(self, message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            if math.isfinite(position):
                self._joint_positions[name] = float(position)
        self._joint_state_received_at = time.monotonic()

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self._requested_enabled = True
            self._outputs_enabled = False
            self._previous_hand_q = {"left": None, "right": None}
            self._last_hand_command_at = 0.0
            response.success = True
            response.message = "hand enable requested; waiting for fresh paired tracking"
            self.get_logger().warning(
                "PICO hand enable requested; body control is not managed by this node"
            )
        else:
            self._disable_outputs("operator request")
            response.success = True
            response.message = "PICO hand output disabled"
        return response

    def _disable_outputs(self, reason: str) -> None:
        was_active = self._requested_enabled or self._outputs_enabled
        self._requested_enabled = False
        self._outputs_enabled = False
        self._previous_hand_q = {"left": None, "right": None}
        self._last_hand_command_at = 0.0
        if was_active:
            self.get_logger().warning(f"PICO hand output disabled: {reason}")

    def _drain_udp(self) -> None:
        while True:
            try:
                datagram, _peer = self._socket.recvfrom(DATAGRAM_SIZE + 1)
            except BlockingIOError:
                return
            except OSError as exc:
                self._log_throttled("error", "udp_receive", f"UDP receive failed: {exc}")
                return
            try:
                frame = decode_frame(datagram)
            except WireError as exc:
                self._invalid_datagrams += 1
                self._log_throttled(
                    "warning",
                    "bad_datagram",
                    f"Rejected PICO hand datagram #{self._invalid_datagrams}: {exc}",
                )
                continue

            non_monotonic = (
                frame.sequence <= self._last_sequence
                or frame.device_timestamp_ns <= self._last_device_timestamp_ns
                or frame.source_timestamp_ns <= self._last_source_timestamp_ns
            )
            source_was_stale = (
                self._latest_received_at > 0.0
                and time.monotonic() - self._latest_received_at > self._source_timeout
            )
            if non_monotonic and source_was_stale and frame.sequence == 0:
                self.get_logger().warning("Accepted a new PICO hand source epoch")
                self._last_sequence = -1
                self._last_device_timestamp_ns = -1
                self._last_source_timestamp_ns = -1
                non_monotonic = False
            if non_monotonic:
                self._out_of_order_datagrams += 1
                self._log_throttled(
                    "warning",
                    "out_of_order",
                    "Dropped non-monotonic PICO hand frame "
                    f"(count={self._out_of_order_datagrams})",
                )
                continue
            self._last_sequence = frame.sequence
            self._last_device_timestamp_ns = frame.device_timestamp_ns
            self._last_source_timestamp_ns = frame.source_timestamp_ns
            self._latest_frame = frame
            self._latest_received_at = time.monotonic()

    def _drain_zmq(self) -> None:
        # Bounded work per ROS tick; the socket keeps only the newest message.
        for _ in range(16):
            try:
                message = self._socket.recv(flags=self._zmq.DONTWAIT)
            except self._zmq.Again:
                return
            except self._zmq.ZMQError as exc:
                self._disable_outputs(f"optical ZMQ receive failed: {exc}")
                self._latest_frame = None
                return
            try:
                session, frame = decode_optical_message(message)
            except WireError as exc:
                self._latest_frame = None
                self._disable_outputs("invalid optical ZMQ frame")
                self._log_throttled("warning", "bad_zmq", f"Rejected optical frame: {exc}")
                continue
            if session != self._source_session:
                if self._source_session is not None:
                    self._disable_outputs("optical source restarted; enable again")
                self._source_session = session
                self._latest_frame = None
                self._last_sequence = -1
                self._last_device_timestamp_ns = -1
                self._last_source_timestamp_ns = -1
                self.get_logger().info("Receiving optical hands from robot manager")
            if (frame.sequence <= self._last_sequence
                    or frame.device_timestamp_ns <= self._last_device_timestamp_ns
                    or frame.source_timestamp_ns <= self._last_source_timestamp_ns):
                self._latest_frame = None
                self._disable_outputs("non-monotonic optical source; enable again")
                continue
            self._last_sequence = frame.sequence
            self._last_device_timestamp_ns = frame.device_timestamp_ns
            self._last_source_timestamp_ns = frame.source_timestamp_ns
            self._latest_frame = frame
            self._latest_received_at = time.monotonic()

    def _measured_native6(self, side: str, now: float) -> np.ndarray | None:
        if now - self._joint_state_received_at > self._joint_state_timeout:
            return None
        names = joint_names(side)
        if any(name not in self._joint_positions for name in names):
            return None
        jtc_positions = np.asarray(
            [self._joint_positions[name] for name in names], dtype=np.float32
        )
        return np.clip(jtc_positions[::-1], NATIVE6_LOWER, NATIVE6_UPPER)

    def _make_trajectory(self, command) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(command.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in command.positions]
        point.time_from_start = _duration(self._trajectory_duration)
        trajectory.points = [point]
        return trajectory

    def _publish_hands(self, frame: PicoUdpFrame, now: float) -> None:
        if frame.left_hand is None or frame.right_hand is None:
            self._log_throttled(
                "warning", "hand_missing", "Optical hand pair incomplete; both hands hold"
            )
            return
        try:
            targets = {
                "left": self._left_calibration.retarget(frame.left_hand),
                "right": self._right_calibration.retarget(frame.right_hand),
            }
        except OpticalRetargetingError as exc:
            self._log_throttled(
                "warning", "hand_retarget", f"Optical hands rejected; both hold: {exc}"
            )
            return

        for side in ("left", "right"):
            if self._previous_hand_q[side] is None:
                measured = self._measured_native6(side, now)
                if measured is None:
                    if self._require_hand_state:
                        self._log_throttled(
                            "warning",
                            "hand_state_missing",
                            "Hands waiting for fresh /joint_states with all 12 actuated joints",
                        )
                        return
                    measured = NATIVE6_LOWER.copy()
                self._previous_hand_q[side] = measured

        elapsed = (
            1.0 / self._publish_rate
            if self._last_hand_command_at <= 0.0
            else now - self._last_hand_command_at
        )
        elapsed = min(max(elapsed, 1e-4), self._trajectory_duration)
        try:
            commands = {
                side: make_jtc_command(
                    side=side,
                    target=targets[side],
                    previous=self._previous_hand_q[side],
                    elapsed_s=elapsed,
                    velocity_margin=self._velocity_margin,
                )
                for side in ("left", "right")
            }
        except HandCommandError as exc:
            self._log_throttled("error", "hand_command", f"Hand command rejected: {exc}")
            return

        for side, command in commands.items():
            self._hand_publishers[side].publish(self._make_trajectory(command))
            self._previous_hand_q[side] = command.native6_positions
        self._last_hand_command_at = now

    def _on_timer(self) -> None:
        if self._hand_zmq_endpoint:
            # Check expiry before accepting a reconnect's packet.
            if (self._latest_received_at > 0.0
                    and time.monotonic() - self._latest_received_at > self._source_timeout):
                self._disable_outputs("PICO hand source stale")
            self._drain_zmq()
        else:
            self._drain_udp()
        now = time.monotonic()
        if self._requested_enabled and not self._outputs_enabled:
            if self._latest_frame is None or now - self._latest_received_at > self._source_timeout:
                self._log_throttled(
                    "warning", "enable_no_source", "Hand enable pending: no fresh PICO frame"
                )
            elif self._latest_frame.left_hand is None or self._latest_frame.right_hand is None:
                self._log_throttled(
                    "warning", "enable_pair_missing", "Hand enable pending: pair incomplete"
                )
            else:
                self._outputs_enabled = True
                self.get_logger().warning(
                    "PICO hand output enabled; official Gear Sonic body remains independent"
                )
        if not self._outputs_enabled:
            return
        if self._latest_frame is None or now - self._latest_received_at > self._source_timeout:
            self._disable_outputs("PICO hand source stale")
            return
        self._publish_hands(self._latest_frame, now)

    def destroy_node(self) -> bool:
        self._disable_outputs("node shutdown")
        self._socket.close()
        if self._zmq_context is not None:
            self._zmq_context.term()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PicoHandAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
