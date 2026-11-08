"""Strict calibrated OpenXR-26 to Inspire native6 retargeting."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any

import numpy as np


OPENXR_HAND_SHAPE = (26, 7)  # xyz + quaternion xyzw
NATIVE6_ORDER = ("pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotation")
NATIVE6_LOWER = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
NATIVE6_UPPER = np.asarray([1.7, 1.7, 1.7, 1.7, 0.6, 1.3], dtype=np.float32)
THUMB_ROTATION_CLOSED_INDEX = np.float32(0.3)
MIN_FEATURE_SPAN = 1e-3


class OpenXRHandJoint(IntEnum):
    PALM = 0
    WRIST = 1
    THUMB_METACARPAL = 2
    THUMB_PROXIMAL = 3
    THUMB_DISTAL = 4
    THUMB_TIP = 5
    INDEX_METACARPAL = 6
    INDEX_PROXIMAL = 7
    INDEX_INTERMEDIATE = 8
    INDEX_DISTAL = 9
    INDEX_TIP = 10
    MIDDLE_METACARPAL = 11
    MIDDLE_PROXIMAL = 12
    MIDDLE_INTERMEDIATE = 13
    MIDDLE_DISTAL = 14
    MIDDLE_TIP = 15
    RING_METACARPAL = 16
    RING_PROXIMAL = 17
    RING_INTERMEDIATE = 18
    RING_DISTAL = 19
    RING_TIP = 20
    LITTLE_METACARPAL = 21
    LITTLE_PROXIMAL = 22
    LITTLE_INTERMEDIATE = 23
    LITTLE_DISTAL = 24
    LITTLE_TIP = 25


_FINGER_CHAINS = {
    "pinky": (21, 22, 23, 24, 25),
    "ring": (16, 17, 18, 19, 20),
    "middle": (11, 12, 13, 14, 15),
    "index": (6, 7, 8, 9, 10),
}
_THUMB_CHAIN = (2, 3, 4, 5)


class OpticalRetargetingError(ValueError):
    """Raised when optical tracking or calibration is unsafe to map."""


def _native6(value: Any, *, name: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError, OverflowError) as exc:
        raise OpticalRetargetingError(f"{name} must be numeric native6") from exc
    if result.shape != (6,):
        raise OpticalRetargetingError(f"{name} must have shape (6,), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise OpticalRetargetingError(f"{name} contains NaN or infinity")
    return np.array(result, dtype=np.float32, copy=True)


def validate_openxr26(hand: Any) -> np.ndarray:
    """Return an owned normalized OpenXR-26 pose array or fail closed."""

    try:
        result = np.asarray(hand, dtype=np.float32)
    except (TypeError, ValueError, OverflowError) as exc:
        raise OpticalRetargetingError("hand must be a numeric OpenXR-26 array") from exc
    if result.shape != OPENXR_HAND_SHAPE:
        raise OpticalRetargetingError(
            f"hand must have shape {OPENXR_HAND_SHAPE}, got {result.shape}"
        )
    if not np.all(np.isfinite(result)):
        raise OpticalRetargetingError("hand contains NaN or infinity")
    owned = np.array(result, dtype=np.float32, copy=True)
    norms = np.linalg.norm(owned[:, 3:], axis=1)
    # The retargeter uses every joint except PALM. XRoboToolkit has no
    # per-joint validity flags, so a zero quaternion is the only available
    # invalid marker. Preserve the source implementation's behaviour: PALM may
    # be absent, but every wrist/finger joint needed by the geometry must exist.
    if np.any(norms[1:] <= 1e-8):
        raise OpticalRetargetingError("hand contains a zero joint quaternion in required joints")
    owned[0, 3:] = (
        owned[0, 3:] / norms[0]
        if norms[0] > 1e-8
        else np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    )
    owned[1:, 3:] /= norms[1:, None]
    return owned


def _unit(vector: np.ndarray, *, name: str) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or norm <= 1e-7:
        raise OpticalRetargetingError(f"{name} is degenerate")
    return vector / norm


def _chain_curl(positions: np.ndarray, chain: tuple[int, ...], *, name: str) -> float:
    bones = np.diff(positions[np.asarray(chain, dtype=np.int64)], axis=0)
    units = np.stack([_unit(bone, name=f"{name} bone") for bone in bones])
    dots = np.sum(units[:-1] * units[1:], axis=1)
    return float(np.sum(np.arccos(np.clip(dots, -1.0, 1.0))))


def _thumb_rotation(positions: np.ndarray) -> float:
    wrist = positions[OpenXRHandJoint.WRIST]
    middle = positions[OpenXRHandJoint.MIDDLE_METACARPAL]
    index = positions[OpenXRHandJoint.INDEX_METACARPAL]
    little = positions[OpenXRHandJoint.LITTLE_METACARPAL]
    thumb_base = positions[OpenXRHandJoint.THUMB_METACARPAL]
    thumb_tip = positions[OpenXRHandJoint.THUMB_TIP]

    forward = _unit(middle - wrist, name="palm forward axis")
    lateral_raw = index - little
    lateral = _unit(
        lateral_raw - forward * float(np.dot(lateral_raw, forward)),
        name="palm lateral axis",
    )
    thumb = _unit(thumb_tip - thumb_base, name="thumb direction")
    forward_component = float(np.dot(thumb, forward))
    lateral_component = float(np.dot(thumb, lateral))
    if np.hypot(forward_component, lateral_component) <= 1e-7:
        raise OpticalRetargetingError("thumb direction has no component in the palm plane")
    return float(np.arctan2(lateral_component, forward_component))


def extract_native6_features(hand: Any) -> np.ndarray:
    """Extract four independent curls, thumb bend, and thumb palm rotation."""

    positions = validate_openxr26(hand)[:, :3]
    features = np.asarray(
        [_chain_curl(positions, _FINGER_CHAINS[name], name=name) for name in NATIVE6_ORDER[:4]]
        + [_chain_curl(positions, _THUMB_CHAIN, name="thumb_bend"), _thumb_rotation(positions)],
        dtype=np.float32,
    )
    if not np.all(np.isfinite(features)):
        raise OpticalRetargetingError("extracted hand features are not finite")
    return features


def apply_thumb_index_collision_envelope(values: Any) -> np.ndarray:
    """Continuously reduce allowed thumb rotation as the index closes."""

    result = _native6(values, name="values")
    if np.any(result < NATIVE6_LOWER) or np.any(result > NATIVE6_UPPER):
        raise OpticalRetargetingError("native6 values are outside Inspire position limits")
    index_closed_fraction = float(result[3] / NATIVE6_UPPER[3])
    allowed = float(NATIVE6_UPPER[5]) + index_closed_fraction * (
        float(THUMB_ROTATION_CLOSED_INDEX) - float(NATIVE6_UPPER[5])
    )
    result[5] = min(float(result[5]), allowed)
    return result


@dataclass(frozen=True)
class OpticalCalibration:
    """Explicit per-hand open/closed feature calibration."""

    side: str
    open_features: np.ndarray
    closed_features: np.ndarray
    open_q: np.ndarray = field(default_factory=lambda: NATIVE6_LOWER.copy())
    closed_q: np.ndarray = field(default_factory=lambda: NATIVE6_UPPER.copy())

    def __post_init__(self) -> None:
        if self.side not in ("left", "right"):
            raise OpticalRetargetingError("side must be left or right")
        open_features = _native6(self.open_features, name="open_features")
        closed_features = _native6(self.closed_features, name="closed_features")
        open_q = _native6(self.open_q, name="open_q")
        closed_q = _native6(self.closed_q, name="closed_q")
        if np.any(open_q < NATIVE6_LOWER) or np.any(open_q > NATIVE6_UPPER):
            raise OpticalRetargetingError("open_q is outside Inspire position limits")
        if np.any(closed_q < NATIVE6_LOWER) or np.any(closed_q > NATIVE6_UPPER):
            raise OpticalRetargetingError("closed_q is outside Inspire position limits")
        degenerate = np.flatnonzero(np.abs(closed_features - open_features) < MIN_FEATURE_SPAN)
        if degenerate.size:
            names = [NATIVE6_ORDER[int(index)] for index in degenerate]
            raise OpticalRetargetingError(f"calibration has degenerate features: {names}")
        for value in (open_features, closed_features, open_q, closed_q):
            value.setflags(write=False)
        object.__setattr__(self, "open_features", open_features)
        object.__setattr__(self, "closed_features", closed_features)
        object.__setattr__(self, "open_q", open_q)
        object.__setattr__(self, "closed_q", closed_q)

    @classmethod
    def capture(
        cls,
        *,
        side: str,
        open_hand: Any,
        closed_hand: Any,
        open_q: Any = NATIVE6_LOWER,
        closed_q: Any = NATIVE6_UPPER,
    ) -> "OpticalCalibration":
        return cls(
            side=side,
            open_features=extract_native6_features(open_hand),
            closed_features=extract_native6_features(closed_hand),
            open_q=open_q,
            closed_q=closed_q,
        )

    def retarget(self, hand: Any) -> np.ndarray:
        features = extract_native6_features(hand)
        fraction = np.clip(
            (features - self.open_features) / (self.closed_features - self.open_features),
            0.0,
            1.0,
        )
        result = self.open_q + fraction * (self.closed_q - self.open_q)
        result = np.clip(result, NATIVE6_LOWER, NATIVE6_UPPER).astype(np.float32)
        return apply_thumb_index_collision_envelope(result)
