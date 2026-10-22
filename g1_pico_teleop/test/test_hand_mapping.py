import numpy as np
import pytest

from g1_pico_teleop.hand.hand_command import HandCommandError, make_jtc_command
from g1_pico_teleop.hand.optical import (
    NATIVE6_UPPER,
    OpticalCalibration,
    OpticalRetargetingError,
    apply_thumb_index_collision_envelope,
    extract_native6_features,
)


def _chain(start, lengths, angles):
    points = [np.asarray(start, dtype=np.float32)]
    for length, angle in zip(lengths, angles):
        direction = np.asarray([0.0, np.cos(angle), -np.sin(angle)], dtype=np.float32)
        points.append(points[-1] + float(length) * direction)
    return np.asarray(points)


def synthetic_hand(*, closed_channels=()) -> np.ndarray:
    hand = np.zeros((26, 7), dtype=np.float32)
    hand[:, 6] = 1.0
    hand[1, :3] = [0.0, -0.06, 0.0]
    specs = {
        "index": (6, 0.03),
        "middle": (11, 0.01),
        "ring": (16, -0.01),
        "pinky": (21, -0.03),
    }
    for name, (start_index, x) in specs.items():
        angles = [0.0, 0.45, 0.95, 1.30] if name in closed_channels else [0.0] * 4
        hand[start_index : start_index + 5, :3] = _chain([x, 0.0, 0.0], [0.025] * 4, angles)

    thumb_planar = np.asarray(
        [-0.15, 0.988, 0.0] if "thumb_rotation" in closed_channels else [0.72, 0.694, 0.0],
        dtype=np.float32,
    )
    thumb_planar /= np.linalg.norm(thumb_planar)
    bend_angles = [0.10, 0.70, 1.20] if "thumb_bend" in closed_channels else [0.0] * 3
    thumb_points = [np.asarray([0.025, -0.005, 0.0], dtype=np.float32)]
    for angle in bend_angles:
        direction = np.cos(angle) * thumb_planar + np.asarray([0.0, 0.0, -np.sin(angle)])
        thumb_points.append(thumb_points[-1] + 0.022 * direction)
    hand[2:6, :3] = np.asarray(thumb_points)
    return hand


ALL_CLOSED = {"pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotation"}


def test_calibrated_retarget_and_collision_envelope() -> None:
    calibration = OpticalCalibration.capture(
        side="left",
        open_hand=synthetic_hand(),
        closed_hand=synthetic_hand(closed_channels=ALL_CLOSED),
    )
    np.testing.assert_allclose(calibration.retarget(synthetic_hand()), 0.0, atol=1e-6)
    closed = calibration.retarget(synthetic_hand(closed_channels=ALL_CLOSED))
    np.testing.assert_allclose(closed[:5], NATIVE6_UPPER[:5], atol=1e-6)
    np.testing.assert_allclose(closed[5], 0.3, atol=1e-6)
    middle = apply_thumb_index_collision_envelope([0.0, 0.0, 0.0, 0.85, 0.0, 1.3])
    np.testing.assert_allclose(middle[5], 0.8, atol=1e-6)


def test_optical_retarget_is_independent_per_finger() -> None:
    calibration = OpticalCalibration.capture(
        side="right",
        open_hand=synthetic_hand(),
        closed_hand=synthetic_hand(closed_channels=ALL_CLOSED),
    )
    index_only = calibration.retarget(synthetic_hand(closed_channels={"index"}))
    assert index_only[3] > 1.6
    np.testing.assert_allclose(index_only[:3], 0.0, atol=1e-6)


def test_optical_rejects_bad_shape_zero_quaternion_and_degenerate_calibration() -> None:
    with pytest.raises(OpticalRetargetingError, match="shape"):
        extract_native6_features(np.zeros((25, 7), dtype=np.float32))
    bad = synthetic_hand()
    bad[4, 3:] = 0.0
    with pytest.raises(OpticalRetargetingError, match="zero joint quaternion"):
        extract_native6_features(bad)
    with pytest.raises(OpticalRetargetingError, match="degenerate"):
        OpticalCalibration.capture(
            side="left", open_hand=synthetic_hand(), closed_hand=synthetic_hand()
        )


def test_unused_palm_may_have_no_orientation() -> None:
    hand = synthetic_hand()
    hand[0, 3:] = 0.0
    features = extract_native6_features(hand)
    assert features.shape == (6,)


def test_native6_reorders_to_ta_jtc_and_limits_slew() -> None:
    command = make_jtc_command(
        side="right",
        target=[1.7, 1.6, 1.5, 1.4, 0.6, 1.3],
        previous=[0.0] * 6,
        elapsed_s=0.1,
        velocity_margin=1.0,
    )
    np.testing.assert_allclose(command.native6_positions, [0.2, 0.2, 0.2, 0.2, 0.1, 0.1])
    np.testing.assert_allclose(command.positions, [0.1, 0.1, 0.2, 0.2, 0.2, 0.2])
    assert command.limited
    assert command.joint_names[0] == "R_thumb_proximal_yaw_joint"
    assert command.joint_names[-1] == "R_pinky_proximal_joint"


def test_hand_command_rejects_invalid_time_and_range() -> None:
    with pytest.raises(HandCommandError, match="elapsed_s"):
        make_jtc_command(side="left", target=[0.0] * 6, previous=[0.0] * 6, elapsed_s=0.0)
    with pytest.raises(HandCommandError, match="position limits"):
        make_jtc_command(side="left", target=[2.0] * 6, previous=[0.0] * 6, elapsed_s=0.1)
