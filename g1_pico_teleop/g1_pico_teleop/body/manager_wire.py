"""Decode the manager's public ZMQ format without importing its robot SDK.

Reference: gear_sonic/utils/teleop/zmq/zmq_planner_sender.py. Messages use
topic + 1280-byte JSON header + little-endian arrays. SMPL coordinates are
already converted by the manager; only quaternion field order changes in ROS.
"""

from dataclasses import dataclass
import json
import math

import numpy as np


HEADER_SIZE = 1280
MAX_MESSAGE_SIZE = 262144
MAX_WINDOW = 64
TOPICS = (b"pose", b"manager_state", b"command")
_DTYPES = {
    "f32": np.dtype("<f4"), "f64": np.dtype("<f8"),
    "i32": np.dtype("<i4"), "i64": np.dtype("<i8"),
    "u8": np.dtype("u1"), "bool": np.dtype("?"),
}


class ManagerWireError(ValueError):
    """Malformed or unsupported manager input."""


@dataclass(frozen=True)
class PoseWindow:
    frame_index: np.ndarray
    smpl_pose: np.ndarray
    smpl_joints: np.ndarray
    body_quat_w: np.ndarray
    joint_pos: np.ndarray
    joint_vel: np.ndarray
    source_stamp: float | None


def _unique_object(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ManagerWireError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _fields(message):
    if not isinstance(message, bytes) or len(message) > MAX_MESSAGE_SIZE:
        raise ManagerWireError("invalid message type or size")
    topic = next((item for item in TOPICS if message.startswith(item)), None)
    if topic is None or len(message) < len(topic) + HEADER_SIZE:
        raise ManagerWireError("missing topic or full header")
    raw = message[len(topic):len(topic) + HEADER_SIZE]
    encoded, separator, padding = raw.partition(b"\0")
    if not separator or any(padding):
        raise ManagerWireError("invalid header padding")
    try:
        header = json.loads(encoded, object_pairs_hook=_unique_object)
    except (ValueError, UnicodeError, RecursionError) as exc:
        raise ManagerWireError(f"invalid header: {exc}") from exc
    if (not isinstance(header, dict) or header.get("endian") != "le"
            or type(header.get("v")) is not int
            or header["v"] != (1 if topic == b"command" else 3)
            or type(header.get("count", 1)) is not int
            or header.get("count", 1) != 1):
        raise ManagerWireError("unsupported version, byte order or count")
    fields = header.get("fields")
    if not isinstance(fields, list) or not 1 <= len(fields) <= 64:
        raise ManagerWireError("invalid field list")
    offset = len(topic) + HEADER_SIZE
    arrays = {}
    for field in fields:
        if not isinstance(field, dict):
            raise ManagerWireError("field must be an object")
        name, dtype, shape = field.get("name"), field.get("dtype"), field.get("shape")
        if (not isinstance(name, str) or not name or name in arrays
                or not isinstance(dtype, str) or dtype not in _DTYPES
                or not isinstance(shape, list) or not 1 <= len(shape) <= 4
                or any(type(size) is not int or not 1 <= size <= 4096 for size in shape)):
            raise ManagerWireError("invalid field name, dtype or dimensions")
        size = math.prod(shape) * _DTYPES[dtype].itemsize
        if offset + size > len(message):
            raise ManagerWireError(f"truncated field: {name}")
        array = np.frombuffer(message, dtype=_DTYPES[dtype], count=math.prod(shape),
                              offset=offset).reshape(shape).copy()
        if not np.all(np.isfinite(array)):
            raise ManagerWireError(f"non-finite field: {name}")
        arrays[name] = array
        offset += size
    if offset != len(message):
        raise ManagerWireError("trailing payload bytes")
    return topic, arrays


def decode_manager_message(message):
    """Return (topic, PoseWindow | mode integer | stop boolean)."""
    topic, arrays = _fields(message)
    if topic == b"command":
        for name in ("start", "stop", "planner"):
            value = arrays.get(name)
            if (value is None or value.shape != (1,) or value.dtype != np.dtype("u1")
                    or int(value[0]) not in (0, 1)):
                raise ManagerWireError(f"invalid command {name}")
        return topic, bool(arrays["stop"][0])
    if topic == b"manager_state":
        mode = arrays.get("stream_mode")
        if mode is None or mode.shape != (1,) or mode.dtype != np.dtype("<i4"):
            raise ManagerWireError("invalid manager stream_mode")
        if int(mode[0]) not in range(6):
            raise ManagerWireError("unknown manager stream_mode")
        return topic, int(mode[0])

    index = arrays.get("frame_index")
    if (index is None or index.ndim != 1 or index.dtype != np.dtype("<i8")
            or not 1 <= len(index) <= MAX_WINDOW or np.any(index < 0)
            or np.any(index[1:] <= index[:-1])):
        raise ManagerWireError("invalid or non-increasing frame_index")
    count = len(index)
    shapes = {"smpl_pose": (count, 21, 3), "smpl_joints": (count, 24, 3),
              "body_quat_w": (count, 4), "joint_pos": (count, 29),
              "joint_vel": (count, 29)}
    for name, shape in shapes.items():
        if name not in arrays and name in ("joint_pos", "joint_vel"):
            arrays[name] = np.zeros(shape, dtype=np.float32)
        value = arrays.get(name)
        if value is None or value.shape != shape or value.dtype.kind != "f":
            raise ManagerWireError(f"invalid {name} shape or dtype")
        if np.any(np.abs(value) > np.finfo(np.float32).max):
            raise ManagerWireError(f"{name} exceeds SONIC float32 range")
    norms = np.linalg.norm(arrays["body_quat_w"], axis=1)
    if np.any(np.abs(norms - 1.0) > 0.01):
        raise ManagerWireError("body quaternion must have unit length")
    stamp = arrays.get("timestamp_monotonic")
    if stamp is not None and (stamp.shape != (1,) or float(stamp[0]) <= 0):
        raise ManagerWireError("invalid source timestamp")
    return topic, PoseWindow(index, *(arrays[name] for name in shapes),
                              None if stamp is None else float(stamp[0]))
