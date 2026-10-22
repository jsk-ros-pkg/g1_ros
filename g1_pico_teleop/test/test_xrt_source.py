import importlib.util
from pathlib import Path
import sys

import numpy as np


def load_source_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "pico_xrt_source.py"
    spec = importlib.util.spec_from_file_location("test_pico_xrt_source", script)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class FakeSdk:
    def __init__(self):
        self.body = np.zeros((24, 7), dtype=np.float32)
        self.body[:, 6] = 1.0
        self.hand = np.zeros((26, 7), dtype=np.float32)
        self.hand[:, 6] = 1.0

    def get_time_stamp_ns(self):
        return 123

    def get_body_joints_pose(self):
        return self.body

    def get_left_hand_is_active(self):
        return False

    def get_left_hand_tracking_state(self):
        return self.hand

    def get_right_hand_is_active(self):
        return True

    def get_right_hand_tracking_state(self):
        return self.hand


def test_missing_one_hand_does_not_block_body_frame():
    module = load_source_module()
    result = module._capture_once(FakeSdk())

    assert result.reason == "ok"
    assert result.body.shape == (24, 7)
    assert result.left_hand is None
    assert result.left_reason == "inactive"
    assert result.right_hand.shape == (26, 7)


def test_hands_only_capture_never_reads_body():
    class HandsOnlySdk(FakeSdk):
        def get_body_joints_pose(self):
            raise AssertionError("hands-only mode must not read body posture")

    module = load_source_module()
    result = module._capture_once(HandsOnlySdk(), include_body=False)

    assert result.reason == "ok"
    np.testing.assert_array_equal(result.body, np.zeros((24, 7), dtype=np.float32))
    assert result.left_hand is None
    assert result.right_hand.shape == (26, 7)
