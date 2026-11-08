"""Map canonical native6 targets into the TA's hand-controller contract."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .optical import NATIVE6_LOWER, NATIVE6_UPPER


NATIVE6_VELOCITY_LIMIT_RAD_S = np.asarray([2.0, 2.0, 2.0, 2.0, 1.0, 1.0], dtype=np.float32)
_JTC_FROM_NATIVE6 = np.asarray([5, 4, 3, 2, 1, 0], dtype=np.int64)


class HandCommandError(ValueError):
    """Raised when a JTC command cannot be produced safely."""


def _validate_native6(value: Any, *, name: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError, OverflowError) as exc:
        raise HandCommandError(f"{name} must be numeric native6") from exc
    if result.shape != (6,):
        raise HandCommandError(f"{name} must have shape (6,), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise HandCommandError(f"{name} contains NaN or infinity")
    if np.any(result < NATIVE6_LOWER) or np.any(result > NATIVE6_UPPER):
        raise HandCommandError(f"{name} is outside Inspire position limits")
    return np.array(result, dtype=np.float32, copy=True)


def joint_names(side: str) -> tuple[str, ...]:
    """Return the exact JointTrajectoryController joint order from g1_ros."""

    if side not in ("left", "right"):
        raise HandCommandError("side must be left or right")
    prefix = "L" if side == "left" else "R"
    return (
        f"{prefix}_thumb_proximal_yaw_joint",
        f"{prefix}_thumb_proximal_pitch_joint",
        f"{prefix}_index_proximal_joint",
        f"{prefix}_middle_proximal_joint",
        f"{prefix}_ring_proximal_joint",
        f"{prefix}_pinky_proximal_joint",
    )


def slew_limit_native6(
    target: Any,
    previous: Any,
    *,
    elapsed_s: float,
    velocity_margin: float = 0.8,
) -> tuple[np.ndarray, bool]:
    """Limit one canonical command step using verified per-joint velocities."""

    if not np.isfinite(elapsed_s) or elapsed_s <= 0.0:
        raise HandCommandError("elapsed_s must be positive and finite")
    if not np.isfinite(velocity_margin) or not 0.0 < velocity_margin <= 1.0:
        raise HandCommandError("velocity_margin must be in (0, 1]")
    target_value = _validate_native6(target, name="target")
    previous_value = _validate_native6(previous, name="previous")
    maximum_delta = NATIVE6_VELOCITY_LIMIT_RAD_S * np.float32(elapsed_s * velocity_margin)
    delta = target_value - previous_value
    limited_delta = np.clip(delta, -maximum_delta, maximum_delta)
    return (previous_value + limited_delta).astype(np.float32), not np.array_equal(
        delta, limited_delta
    )


@dataclass(frozen=True)
class JtcHandCommand:
    """ROS-independent content for one JointTrajectory command point."""

    side: str
    joint_names: tuple[str, ...]
    positions: np.ndarray
    native6_positions: np.ndarray
    limited: bool

    def __post_init__(self) -> None:
        expected_names = joint_names(self.side)
        if tuple(self.joint_names) != expected_names:
            raise HandCommandError("joint_names do not match the g1_ros hand controller order")
        positions = np.asarray(self.positions, dtype=np.float32)
        if positions.shape != (6,) or not np.all(np.isfinite(positions)):
            raise HandCommandError("positions must be finite shape (6,)")
        native = _validate_native6(self.native6_positions, name="native6_positions")
        positions = np.array(positions, dtype=np.float32, copy=True)
        positions.setflags(write=False)
        native.setflags(write=False)
        object.__setattr__(self, "positions", positions)
        object.__setattr__(self, "native6_positions", native)


def make_jtc_command(
    *,
    side: str,
    target: Any,
    previous: Any,
    elapsed_s: float,
    velocity_margin: float = 0.8,
) -> JtcHandCommand:
    """Slew-limit native6 and reorder it for the existing TA JTC."""

    limited_native, limited = slew_limit_native6(
        target,
        previous,
        elapsed_s=elapsed_s,
        velocity_margin=velocity_margin,
    )
    return JtcHandCommand(
        side=side,
        joint_names=joint_names(side),
        positions=limited_native[_JTC_FROM_NATIVE6],
        native6_positions=limited_native,
        limited=limited,
    )
