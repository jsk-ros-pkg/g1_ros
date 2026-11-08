"""Flood coalescing must preserve control events and their receive times."""

import numpy as np

from g1_pico_teleop.body.manager_receiver import ManagerInbox
from test_manager_wire import DATA, pack


def state(mode):
    return pack(b"manager_state", {"stream_mode": np.array([mode], dtype=np.int32)})


def test_repeated_off_heartbeats_do_not_overflow():
    inbox = ManagerInbox(capacity=4)
    off = state(0)
    for index in range(10000):
        inbox.put(off, float(index))
    messages, fault = inbox.take()
    assert fault is None
    assert messages == [(off, 9999.0)]


def test_stop_and_mode_transitions_survive_repeated_states():
    inbox = ManagerInbox()
    off, planner, unsupported = state(0), state(2), state(5)
    stop = (DATA / "manager_stop_v1.bin").read_bytes()
    events = [off] * 1000 + [planner] * 1000 + [unsupported, planner, stop] + [off] * 1000
    for index, message in enumerate(events):
        inbox.put(message, float(index))
    messages, fault = inbox.take()
    assert fault is None
    assert [message for message, stamp in messages] == [off, planner, unsupported, planner, stop, off]
    assert [stamp for message, stamp in messages] == [999.0, 1999.0, 2000.0, 2001.0, 2002.0, 3002.0]


def test_repeated_poses_remain_visible_to_duplicate_frame_watchdog():
    inbox = ManagerInbox()
    pose = (DATA / "manager_pose_v3.bin").read_bytes()
    inbox.put(pose, 1.0)
    inbox.put(pose, 2.0)
    messages, fault = inbox.take()
    assert fault is None
    assert messages == [(pose, 1.0), (pose, 2.0)]


def test_losing_distinct_events_latches_fault_until_ros_observes_it():
    inbox = ManagerInbox(capacity=2)
    inbox.put(state(0), 1.0)
    inbox.put((DATA / "manager_stop_v1.bin").read_bytes(), 2.0)
    inbox.put(state(2), 3.0)
    inbox.put(state(2), 4.0)
    messages, fault = inbox.take()
    assert "control data was lost" in fault
    assert messages == [(state(2), 4.0)]
    assert inbox.take() == ([], None)
