"""ROS/ZMQ loopback integration; services are fakes in localhost domain 215."""

import os
import subprocess
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from std_srvs.srv import SetBool, Trigger
from gear_sonic_interfaces.msg import SmplMotion
import zmq

from g1_pico_teleop.body.body_ros_adapter_node import PicoBodyAdapter, validate_endpoint
from test_manager_wire import DATA, pack, poses


class Harness:
    def __init__(self, monkeypatch):
        self.context = zmq.Context()
        self.pub = self.context.socket(zmq.PUB)
        self.pub.setsockopt(zmq.LINGER, 0)
        port = self.pub.bind_to_random_port("tcp://127.0.0.1")
        original = Node.__init__
        def init(node, name, **kwargs):
            if name == "pico_body_adapter":
                kwargs["parameter_overrides"] = [
                    Parameter("body_zmq_endpoint", value=f"tcp://127.0.0.1:{port}"),
                    Parameter("source_timeout", value=0.25),
                    Parameter("service_timeout", value=0.2),
                    Parameter("balance_duration", value=0.06),
                    Parameter("pose_start_timeout", value=0.3),
                ]
            original(node, name, **kwargs)
        monkeypatch.setattr(Node, "__init__", init)
        self.node = PicoBodyAdapter()
        self.observer = Node("body_test_observer")
        self.calls, self.messages = [], []
        self.reject_control = False
        self.services = []
        for name, suffix, kind in (("mode", "mode/default", Trigger), ("control", "enable_control", SetBool),
                                   ("stream", "enable_smpl_stream", SetBool)):
            def callback(request, response, name=name):
                value = getattr(request, "data", True)
                self.calls.append((name, value))
                response.success = not (name == "control" and value and self.reject_control)
                response.message = "fake service response"
                return response
            self.services.append(self.observer.create_service(kind, f"/gear_sonic_interface/{suffix}", callback))
        self.subscription = self.observer.create_subscription(SmplMotion, "/gear_sonic_interface/smpl_motion", self.messages.append, 64)
        self.enable_client = self.observer.create_client(SetBool, "/pico_body_adapter/enable")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.observer)
        self.mode, self.counter = 0, 40
        self.send_state, self.send_pose = True, True
        self.state_repeats = 1
        self.freeze_pose = False
        self.last_pose = None
        self.next_send = 0.0
        self.pump(0.35)

    def step(self):
        now = time.monotonic()
        if now >= self.next_send:
            if self.mode == 1 and self.send_pose:
                if not self.freeze_pose or self.last_pose is None:
                    self.last_pose = pack(b"pose", poses(self.counter, stamp=float(self.counter + 1000)))
                    self.counter += 1
                self.pub.send(self.last_pose)
            if self.send_state:
                state = pack(b"manager_state", {"stream_mode": np.array([self.mode], dtype=np.int32)})
                for _ in range(self.state_repeats):
                    self.pub.send(state)
            self.next_send = now + 0.02
        self.executor.spin_once(timeout_sec=0.002)

    def pump(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.step()

    def until(self, predicate, timeout=2.0):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            if predicate(): return
            self.step()
        assert predicate(), "condition did not become true"

    def enable(self, value=True):
        future = self.enable_client.call_async(SetBool.Request(data=value))
        self.until(future.done)
        return future.result()

    def balance(self):
        assert self.enable().success
        self.mode = 2
        self.until(lambda: ("control", True) in self.calls and self.node._rpc is None)

    def follow(self):
        self.balance()
        self.mode = 1
        self.until(lambda: bool(self.messages))

    def stopped(self):
        return not self.node._armed and not self.node._recovering and self.node._rpc is None

    def close(self):
        self.node.request_stop("test shutdown")
        self.pump(0.3)
        self.executor.remove_node(self.node)
        self.executor.remove_node(self.observer)
        self.node.close()
        self.observer.destroy_node()
        self.executor.shutdown()
        self.pub.close(0)
        self.context.term()


@pytest.fixture
def harness(monkeypatch):
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")
    monkeypatch.delenv("ROS_DISCOVERY_SERVER", raising=False)
    monkeypatch.delenv("ROS_STATIC_PEERS", raising=False)
    rclpy.init(domain_id=215)
    h = None
    try:
        h = Harness(monkeypatch)
        yield h
    finally:
        try:
            if h is not None:
                h.close()
        finally:
            rclpy.try_shutdown()


def test_default_disabled_and_require_off_before_arming(harness):
    h = harness
    h.mode = 1
    h.pump(0.15)
    assert not h.calls and not h.messages
    response = h.enable()
    assert not response.success and "OFF" in response.message
    assert not h.calls


def test_off_state_flood_keeps_enable_responsive_then_allows_balance(harness):
    h = harness
    h.state_repeats = 2048
    h.pump(0.3)
    assert h.enable().success  # actual ROS service response, not just the armed flag
    h.pump(0.3)
    assert h.node._armed and not h.calls
    h.state_repeats = 1
    h.mode = 2
    h.until(lambda: ("control", True) in h.calls and h.node._rpc is None)
    h.mode = 1
    h.until(lambda: bool(h.messages))
    h.pub.send((DATA / "manager_stop_v1.bin").read_bytes())
    h.until(h.stopped)
    assert h.calls[-2:] == [("stream", False), ("control", False)]


def test_separate_cli_gets_arm_response_during_off_state_flood(harness, tmp_path):
    h = harness
    h.state_repeats = 2048
    env = dict(os.environ, ROS_DOMAIN_ID="215")
    path = tmp_path / "service-call.log"
    with path.open("w") as output:
        process = subprocess.Popen(
            ["ros2", "service", "call", "/pico_body_adapter/enable",
             "std_srvs/srv/SetBool", "{data: true}"],
            env=env, stdout=output, stderr=subprocess.STDOUT,
        )
        try:
            h.until(lambda: process.poll() is not None, timeout=8.0)
            assert process.returncode == 0, path.read_text()
            assert "success=True" in path.read_text()
            assert "body armed" in path.read_text()
            assert h.node._armed and not h.calls
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)


def test_executor_pause_does_not_make_queued_state_fresh(harness):
    h = harness
    h.follow()
    h.send_state, h.send_pose = False, False
    h.pub.send(pack(b"manager_state", {"stream_mode": np.array([2], dtype=np.int32)}))
    # ZMQ continues receiving while the ROS executor is stalled.
    time.sleep(0.35)
    h.until(h.stopped)
    assert h.node._mode == 1  # the delayed transition was rejected
    assert h.calls[-2:] == [("stream", False), ("control", False)]


def test_follow_mapping_pause_to_balance_and_restart_frame_counter(harness):
    h = harness
    h.follow()
    h.pump(0.12)
    indices = [message.frame_index for message in h.messages]
    assert indices == sorted(set(indices))
    first = h.messages[0]
    assert first.body_quat.w == pytest.approx(2**-0.5)
    assert first.body_quat.z == pytest.approx(2**-0.5)
    assert first.body_quat.x == 0
    assert len(first.smpl_pose) == 63 and len(first.smpl_joints) == 72
    assert h.calls[:3] == [("mode", True), ("control", True), ("stream", True)]
    h.mode = 2
    h.until(lambda: ("stream", False) in h.calls and h.node._rpc is None)
    h.pump(0.1)
    assert h.node._armed  # returning to PLANNER must not trip the pose watchdog
    h.messages.clear()
    h.counter = 0
    h.mode = 1
    h.until(lambda: bool(h.messages))
    assert h.messages[0].frame_index < indices[-1]
    assert h.enable(False).success
    h.until(h.stopped)
    assert h.calls[-2:] == [("stream", False), ("control", False)]


@pytest.mark.parametrize("fault", ["duplicate_pose", "no_state", "no_pose", "bad_message", "frame_rollback", "timestamp_rollback", "manager_restart", "stop_command", "unsupported_mode"])
def test_faults_return_to_balance_without_automatic_reenable(harness, fault):
    h = harness
    h.follow()
    if fault == "duplicate_pose": h.freeze_pose = True
    if fault == "no_state": h.send_state = False
    if fault == "no_pose": h.send_pose = False
    if fault == "bad_message": h.pub.send(b"posebroken")
    if fault == "frame_rollback": h.counter = 0
    if fault == "timestamp_rollback": h.pub.send(pack(b"pose", poses(h.counter + 100, stamp=1.0)))
    if fault == "manager_restart": h.mode = 0
    if fault == "stop_command": h.pub.send((DATA / "manager_stop_v1.bin").read_bytes())
    if fault == "unsupported_mode": h.mode = 5
    h.until(h.stopped)
    assert h.calls[-2:] == [("stream", False), ("control", False)]
    h.send_state, h.send_pose, h.freeze_pose = True, True, False
    h.mode = 0
    h.pump(0.1)
    count, calls = len(h.messages), list(h.calls)
    h.mode = 1
    h.pump(0.15)
    assert len(h.messages) == count and h.calls == calls


def test_control_service_rejection_never_starts_stream(harness):
    h = harness
    h.reject_control = True
    assert h.enable().success
    h.mode = 2
    h.until(lambda: ("control", True) in h.calls)
    h.until(h.stopped)
    assert ("stream", True) not in h.calls and not h.messages


def test_pose_start_without_data_returns_to_balance(harness):
    h = harness
    h.balance()
    h.send_pose = False
    h.mode = 1
    h.until(h.stopped)
    assert ("stream", True) not in h.calls
    assert h.calls[-1] == ("control", False)
    assert not h.messages


def test_late_control_ack_is_compensated_after_timeout(harness, monkeypatch):
    h = harness
    future = Future()
    client = h.node._sonic_clients["control"]
    original = client.call_async
    def delayed(request):
        if request.data:
            h.calls.append(("control", True))
            return future
        return original(request)
    monkeypatch.setattr(client, "call_async", delayed)
    assert h.enable().success
    h.mode = 2
    h.until(lambda: h.node._rpc is not None and h.node._rpc[4])
    assert not h.node._armed
    assert ("control", False) not in h.calls  # serialize cleanup after the pending enable
    future.set_result(SetBool.Response(success=True))
    h.until(h.stopped)
    assert h.calls[-1] == ("control", False)
    assert not h.messages


def test_disable_during_stream_enable_never_forwards_after_late_ack(harness, monkeypatch):
    h = harness
    h.balance()
    future = Future()
    client = h.node._sonic_clients["stream"]
    original = client.call_async
    def delayed(request):
        if request.data:
            h.calls.append(("stream", True))
            return future
        return original(request)
    monkeypatch.setattr(client, "call_async", delayed)
    h.mode = 1
    h.until(lambda: ("stream", True) in h.calls)
    assert h.enable(False).success
    future.set_result(SetBool.Response(success=True))
    h.until(h.stopped)
    assert not h.messages
    assert h.calls[-2:] == [("stream", False), ("control", False)]


@pytest.mark.parametrize("endpoint", ["tcp://127.0.0.1:5556", "tcp://127.0.0.1:5557",
                                     "tcp://127.0.0.1:5560", "ipc:///tmp/input", "tcp://localhost"])
def test_reject_shared_or_invalid_input_endpoints(endpoint):
    with pytest.raises(ValueError):
        validate_endpoint(endpoint)
