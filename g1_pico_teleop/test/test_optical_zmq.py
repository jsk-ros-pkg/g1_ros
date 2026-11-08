"""Self-contained optical transport and ROS loopback tests; no robot required."""
import socket
import time
import uuid

import numpy as np
import pytest
import zmq

from g1_pico_teleop.wire import (
    PicoUdpFrame, WireError, decode_optical_message, encode_frame, OPTICAL_TOPIC,
)
from test_hand_mapping import synthetic_hand


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def make_message(session, sequence, stamp, left, right):
    frame = PicoUdpFrame(sequence, stamp, time.monotonic_ns(),
                         np.zeros((24, 7)), left, right)
    return OPTICAL_TOPIC + session + encode_frame(frame)


@pytest.mark.parametrize('fault', ['prefix','short','long','crc','zero_session','zero_stamp','body'])
def test_wire_rejects_invalid_optical_envelopes(fault):
    msg = make_message(uuid.uuid4().bytes, 0, 100, synthetic_hand(), None)
    if fault == 'prefix': msg = b'WRONG' + msg[5:]
    if fault == 'short': msg = msg[:-1]
    if fault == 'long': msg += b'0'
    if fault == 'crc': msg = msg[:-1] + bytes([msg[-1] ^ 1])
    if fault == 'zero_session': msg = msg[:5] + bytes(16) + msg[21:]
    if fault in ('zero_stamp', 'body'):
        frame = PicoUdpFrame(0, 0 if fault == 'zero_stamp' else 1, 1,
                            np.ones((24, 7)) if fault == 'body' else np.zeros((24, 7)))
        msg = msg[:21] + encode_frame(frame)
    with pytest.raises(WireError): decode_optical_message(msg)



@pytest.fixture
def ros(monkeypatch):
    # Never discover/connect to the real robot's ROS domain 164.
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.delenv('ROS_DISCOVERY_SERVER', raising=False)
    import rclpy
    rclpy.init(domain_id=213)
    yield rclpy
    rclpy.shutdown()


def test_actual_ros_adapter_zmq_publish_restart_timeout_and_udp(ros, monkeypatch):
    from g1_pico_teleop.hand.hand_ros_adapter_node import PicoHandAdapter, _HAND_TOPICS
    from g1_pico_teleop.hand.hand_command import joint_names
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory
    from std_srvs.srv import SetBool
    import rclpy.node
    from rclpy.parameter import Parameter
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.LINGER, 0)
    port = pub.bind_to_random_port('tcp://127.0.0.1')
    # Inject ROS parameter overrides without changing production node API.
    original_init = rclpy.node.Node.__init__
    def init(node, name, **kw):
        if name == 'pico_hand_adapter':
            kw['parameter_overrides'] = [Parameter('hand_zmq_endpoint', value=f'tcp://127.0.0.1:{port}')]
        original_init(node, name, **kw)
    monkeypatch.setattr(rclpy.node.Node, '__init__', init)
    node = PicoHandAdapter()
    observer = ros.create_node('optical_test_observer')
    messages = {s: [] for s in _HAND_TOPICS}
    subs = [observer.create_subscription(JointTrajectory, topic, messages[side].append, 10)
            for side, topic in _HAND_TOPICS.items()]
    feedback = observer.create_publisher(JointState, '/joint_states', 10)
    state = JointState(name=joint_names('left') + joint_names('right'), position=[0.0]*12)
    session = uuid.uuid4().bytes
    seq = 0
    def pump(duration=0.15, send=True, left=True):
        nonlocal seq
        until = time.monotonic() + duration
        while time.monotonic() < until:
            feedback.publish(state)
            if send:
                seq += 1
                hand = synthetic_hand()
                pub.send(make_message(session, seq, seq+10000, hand if left else None, hand))
            ros.spin_once(node, timeout_sec=0.002)
            ros.spin_once(observer, timeout_sec=0.002)
    try:
        pump(0.5)
        assert node._latest_frame is not None
        assert not node._requested_enabled
        assert not any(messages.values())
        node._on_enable(SetBool.Request(data=True), SetBool.Response())
        pump(0.3)
        assert node._outputs_enabled
        assert all(messages.values())  # actual ROS messages received in isolated domain
        for side in messages:
            assert tuple(messages[side][-1].joint_names) == joint_names(side)
        pump(0.1, left=False)
        counts = [len(m) for m in messages.values()]
        pump(0.1, left=False)
        assert [len(m) for m in messages.values()] == counts  # paired-hand hold
        session = uuid.uuid4().bytes
        seq = 0  # seq0 need not be seen by SUB after a PUB restart
        pump(0.15)
        assert not node._requested_enabled and not node._outputs_enabled
        counts = [len(m) for m in messages.values()]
        pump(0.1)
        assert [len(m) for m in messages.values()] == counts
        node._on_enable(SetBool.Request(data=True), SetBool.Response())
        pump(0.1)
        assert node._outputs_enabled
        pump(0.7, send=False)
        assert not node._requested_enabled and not node._outputs_enabled
        pump(0.1)
        assert not node._outputs_enabled  # no auto-reenable after reconnect
        # Regression within same session invalidates control.
        node._on_enable(SetBool.Request(data=True), SetBool.Response())
        pump(0.1)
        seq = 0
        pump(0.1)
        assert not node._outputs_enabled and not node._requested_enabled
    finally:
        node.destroy_node()
        observer.destroy_node()
        pub.close(0)
        ctx.term()
    # Default UDP transport still reads original G1PT bytes.
    def udp_init(node, name, **kw):
        if name == 'pico_hand_adapter':
            kw['parameter_overrides'] = [Parameter('listen_port', value=free_port())]
        original_init(node, name, **kw)
    monkeypatch.setattr(rclpy.node.Node, '__init__', udp_init)
    node = PicoHandAdapter()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        frame = PicoUdpFrame(0, 1, 1, np.zeros((24,7)), synthetic_hand(), synthetic_hand())
        sender.sendto(encode_frame(frame), ('127.0.0.1', node._listen_port))
        node._on_timer()
        assert node._latest_frame is not None
        assert node._zmq_context is None
    finally:
        sender.close()
        node.destroy_node()
