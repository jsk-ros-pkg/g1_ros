"""Fixed-size Pico optical-tracking UDP wire protocol.

Version 1 deliberately uses only a fixed little-endian header, float32 payload
slots, and CRC32.  Hand slots are always present on the wire; flags distinguish
an absent hand from a valid all-zero hand without making datagram length vary.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Any
import zlib

import numpy as np


MAGIC = b"G1PT"
VERSION = 1

BODY_SHAPE = (24, 7)
HAND_SHAPE = (26, 7)

FLAG_LEFT_HAND = 1 << 0
FLAG_RIGHT_HAND = 1 << 1
_KNOWN_FLAGS = FLAG_LEFT_HAND | FLAG_RIGHT_HAND

# magic, version, flags, reserved, sequence, device timestamp, source timestamp
_HEADER = struct.Struct("<4sBBHQQQ")
_CRC = struct.Struct("<I")
_FLOAT_DTYPE = np.dtype("<f4")
_BODY_BYTES = int(np.prod(BODY_SHAPE)) * _FLOAT_DTYPE.itemsize
_HAND_BYTES = int(np.prod(HAND_SHAPE)) * _FLOAT_DTYPE.itemsize
DATAGRAM_SIZE = _HEADER.size + _BODY_BYTES + 2 * _HAND_BYTES + _CRC.size

# Single-part ZMQ envelope; compatible G1PT payload, unused body slot all zero.
OPTICAL_TOPIC = b"G1OH1"
OPTICAL_HEADER_SIZE = len(OPTICAL_TOPIC) + 16  # publisher session UUID
OPTICAL_MESSAGE_SIZE = OPTICAL_HEADER_SIZE + DATAGRAM_SIZE


class WireError(ValueError):
    """Raised when a frame cannot satisfy the v1 datagram contract."""


def decode_optical_message(message: bytes) -> tuple[bytes, PicoUdpFrame]:
    """Validate the robot's optical-only ZMQ envelope and existing v1 frame."""
    if len(message) != OPTICAL_MESSAGE_SIZE or not message.startswith(OPTICAL_TOPIC):
        raise WireError("invalid optical ZMQ envelope length or topic")
    session = message[len(OPTICAL_TOPIC):OPTICAL_HEADER_SIZE]
    frame = decode_frame(message[OPTICAL_HEADER_SIZE:])
    if not any(session) or np.any(frame.body) or not frame.device_timestamp_ns or not frame.source_timestamp_ns:
        raise WireError("invalid optical session, padding or timestamp")
    return session, frame


def _uint64(value: Any, *, field_name: str) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise WireError(f"{field_name} must be an unsigned 64-bit integer")
    result = int(value)
    if result < 0 or result > 0xFFFFFFFFFFFFFFFF:
        raise WireError(f"{field_name} must be an unsigned 64-bit integer")
    return result


def _finite_array(value: Any, shape: tuple[int, int], *, field_name: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError, OverflowError) as exc:
        raise WireError(f"{field_name} must be a float32 array") from exc
    if result.shape != shape:
        raise WireError(f"{field_name} must have shape {shape}, got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise WireError(f"{field_name} contains NaN or infinity")
    owned = np.array(result, dtype=np.float32, copy=True, order="C")
    owned.setflags(write=False)
    return owned


@dataclass(frozen=True)
class PicoUdpFrame:
    """One synchronized body frame with independently optional optical hands."""

    sequence: int
    device_timestamp_ns: int
    source_timestamp_ns: int
    body: np.ndarray
    left_hand: np.ndarray | None = None
    right_hand: np.ndarray | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "sequence", _uint64(self.sequence, field_name="sequence"))
        object.__setattr__(
            self,
            "device_timestamp_ns",
            _uint64(self.device_timestamp_ns, field_name="device_timestamp_ns"),
        )
        object.__setattr__(
            self,
            "source_timestamp_ns",
            _uint64(self.source_timestamp_ns, field_name="source_timestamp_ns"),
        )
        object.__setattr__(self, "body", _finite_array(self.body, BODY_SHAPE, field_name="body"))
        if self.left_hand is not None:
            object.__setattr__(
                self,
                "left_hand",
                _finite_array(self.left_hand, HAND_SHAPE, field_name="left_hand"),
            )
        if self.right_hand is not None:
            object.__setattr__(
                self,
                "right_hand",
                _finite_array(self.right_hand, HAND_SHAPE, field_name="right_hand"),
            )


def _float_bytes(value: np.ndarray) -> bytes:
    return value.astype(_FLOAT_DTYPE, copy=False).tobytes(order="C")


def encode_frame(frame: PicoUdpFrame) -> bytes:
    """Serialize exactly one :class:`PicoUdpFrame` into a v1 datagram."""

    if not isinstance(frame, PicoUdpFrame):
        raise WireError("frame must be a PicoUdpFrame")
    flags = 0
    if frame.left_hand is not None:
        flags |= FLAG_LEFT_HAND
    if frame.right_hand is not None:
        flags |= FLAG_RIGHT_HAND
    header = _HEADER.pack(
        MAGIC,
        VERSION,
        flags,
        0,
        frame.sequence,
        frame.device_timestamp_ns,
        frame.source_timestamp_ns,
    )
    left = bytes(_HAND_BYTES) if frame.left_hand is None else _float_bytes(frame.left_hand)
    right = bytes(_HAND_BYTES) if frame.right_hand is None else _float_bytes(frame.right_hand)
    content = header + _float_bytes(frame.body) + left + right
    checksum = zlib.crc32(content) & 0xFFFFFFFF
    datagram = content + _CRC.pack(checksum)
    if len(datagram) != DATAGRAM_SIZE:  # Defensive invariant for protocol edits.
        raise AssertionError("v1 datagram layout is not fixed length")
    return datagram


def _decode_array(data: bytes, offset: int, shape: tuple[int, int]) -> tuple[np.ndarray, int]:
    byte_count = int(np.prod(shape)) * _FLOAT_DTYPE.itemsize
    value = np.frombuffer(data, dtype=_FLOAT_DTYPE, count=int(np.prod(shape)), offset=offset)
    result = np.array(value.reshape(shape), dtype=np.float32, copy=True)
    return result, offset + byte_count


def decode_frame(datagram: bytes | bytearray | memoryview) -> PicoUdpFrame:
    """Validate and deserialize one complete v1 UDP datagram."""

    if not isinstance(datagram, (bytes, bytearray, memoryview)):
        raise WireError("datagram must be bytes-like")
    data = bytes(datagram)
    if len(data) != DATAGRAM_SIZE:
        raise WireError(f"datagram must be exactly {DATAGRAM_SIZE} bytes, got {len(data)}")

    expected_crc = _CRC.unpack_from(data, len(data) - _CRC.size)[0]
    actual_crc = zlib.crc32(data[:-_CRC.size]) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise WireError("datagram CRC32 mismatch")

    magic, version, flags, reserved, sequence, device_ts, source_ts = _HEADER.unpack_from(data)
    if magic != MAGIC:
        raise WireError("datagram magic mismatch")
    if version != VERSION:
        raise WireError(f"unsupported datagram version {version}")
    if flags & ~_KNOWN_FLAGS:
        raise WireError(f"datagram has unknown flags 0x{flags:02x}")
    if reserved != 0:
        raise WireError("datagram reserved field must be zero")

    offset = _HEADER.size
    body, offset = _decode_array(data, offset, BODY_SHAPE)
    left, offset = _decode_array(data, offset, HAND_SHAPE)
    right, offset = _decode_array(data, offset, HAND_SHAPE)
    if offset != len(data) - _CRC.size:
        raise AssertionError("v1 decoder consumed the wrong payload length")

    if not flags & FLAG_LEFT_HAND:
        if np.any(left.view(np.uint8)):
            raise WireError("absent left hand slot must contain only zero padding")
        left = None
    if not flags & FLAG_RIGHT_HAND:
        if np.any(right.view(np.uint8)):
            raise WireError("absent right hand slot must contain only zero padding")
        right = None

    # PicoUdpFrame performs the final shape and finite-value validation, including
    # catching a NaN payload whose CRC was recomputed by a malformed sender.
    return PicoUdpFrame(
        sequence=sequence,
        device_timestamp_ns=device_ts,
        source_timestamp_ns=source_ts,
        body=body,
        left_hand=left,
        right_hand=right,
    )
