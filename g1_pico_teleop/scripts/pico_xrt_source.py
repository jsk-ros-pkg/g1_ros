#!/usr/bin/env python3
"""Read atomic XRoboToolkit frames and forward them over the Pico UDP v1 wire.

This process intentionally has no ROS dependency.  It is meant to run with the
Python 3.10 environment required by the binary ``xrobotoolkit_sdk`` module,
while the ROS 2 Jazzy adapter receives the datagrams in its own process.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import importlib
import importlib.util
import logging
import math
from pathlib import Path
import signal
import socket
import sys
import threading
import time
from types import ModuleType
from typing import Any

import numpy as np


LOGGER = logging.getLogger("pico_xrt_source")
BODY_SHAPE = (24, 7)
HAND_SHAPE = (26, 7)


def _load_wire_module() -> ModuleType:
    """Load wire.py from an installed package or a checkout/install tree.

    A normal package import is preferred.  The path fallbacks are needed when
    this script is launched with Python 3.10 from a ROS installation whose
    Python package directory is versioned for Python 3.12.
    """

    try:
        return importlib.import_module("g1_pico_teleop.wire")
    except ModuleNotFoundError as package_error:
        script = Path(__file__).resolve()
        candidates = [
            script.with_name("wire.py"),
            script.parent.parent / "g1_pico_teleop" / "wire.py",
            script.parent.parent.parent
            / "share"
            / "g1_pico_teleop"
            / "g1_pico_teleop"
            / "wire.py",
        ]
        install_prefix = script.parent.parent.parent
        candidates.extend(
            sorted(
                install_prefix.glob(
                    "lib/python*/site-packages/g1_pico_teleop/wire.py"
                )
            )
        )
        for candidate in candidates:
            if not candidate.is_file():
                continue
            spec = importlib.util.spec_from_file_location(
                "_g1_pico_teleop_wire", candidate
            )
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            spec.loader.exec_module(module)
            return module
        searched = ", ".join(str(path) for path in candidates)
        raise RuntimeError(
            "cannot import g1_pico_teleop.wire or locate wire.py; searched: "
            f"{searched}"
        ) from package_error


WIRE = _load_wire_module()
PicoUdpFrame = WIRE.PicoUdpFrame
encode_frame = WIRE.encode_frame


@dataclass
class CaptureResult:
    body: np.ndarray | None
    left_hand: np.ndarray | None
    right_hand: np.ndarray | None
    device_timestamp_ns: int
    reason: str = "ok"
    left_reason: str = "ok"
    right_reason: str = "ok"


@dataclass
class Counters:
    polls: int = 0
    sent: int = 0
    unavailable: int = 0
    timestamp_mismatch: int = 0
    duplicate_timestamp: int = 0
    body_rejected: int = 0
    left_missing: int = 0
    right_missing: int = 0
    send_errors: int = 0


def _finite_tracking_array(
    value: Any, shape: tuple[int, int], *, field_name: str
) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{field_name} is not a float32 array") from exc
    if array.shape != shape:
        raise ValueError(f"{field_name} must have shape {shape}, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{field_name} contains NaN or infinity")
    return np.array(array, dtype=np.float32, copy=True, order="C")


def _read_hand(sdk: ModuleType, side: str) -> tuple[np.ndarray | None, str]:
    """Read one hand without allowing its failure to invalidate the body."""

    active = False
    active_error: Exception | None = None
    state: Any = None
    state_error: Exception | None = None
    try:
        active = bool(getattr(sdk, f"get_{side}_hand_is_active")())
    except Exception as exc:
        active_error = exc
    try:
        state = getattr(sdk, f"get_{side}_hand_tracking_state")()
    except Exception as exc:  # Keep reading the other hand and final timestamp.
        state_error = exc

    if active_error is not None:
        return None, f"active_error:{type(active_error).__name__}"
    if state_error is not None:
        return None, f"state_error:{type(state_error).__name__}"
    if not active:
        return None, "inactive"
    try:
        return (
            _finite_tracking_array(state, HAND_SHAPE, field_name=f"{side}_hand"),
            "ok",
        )
    except ValueError as exc:
        return None, str(exc)


def _capture_once(sdk: ModuleType, *, include_body: bool = True) -> CaptureResult:
    """Perform one timestamp-consistent SDK read.

    ``include_body=False`` is the hand-only integration mode. The official
    Gear Sonic PICO manager remains the body owner, so this process does not
    read or validate the body skeleton. A finite zero body is returned only as
    padding for the backwards-compatible Pico UDP v1 envelope; the hand-only
    ROS adapter never consumes that field.
    """

    try:
        timestamp_before = int(sdk.get_time_stamp_ns())
    except Exception as exc:
        return CaptureResult(None, None, None, 0, f"timestamp_before:{type(exc).__name__}")
    if timestamp_before <= 0:
        return CaptureResult(None, None, None, 0, "timestamp_not_ready")

    body_raw: Any = np.zeros(BODY_SHAPE, dtype=np.float32)
    body_error: Exception | None = None
    if include_body:
        try:
            body_raw = sdk.get_body_joints_pose()
        except Exception as exc:
            body_error = exc

    left_hand, left_reason = _read_hand(sdk, "left")
    right_hand, right_reason = _read_hand(sdk, "right")

    try:
        timestamp_after = int(sdk.get_time_stamp_ns())
    except Exception as exc:
        return CaptureResult(
            None,
            None,
            None,
            0,
            f"timestamp_after:{type(exc).__name__}",
            left_reason,
            right_reason,
        )
    if timestamp_after != timestamp_before:
        return CaptureResult(
            None,
            None,
            None,
            timestamp_after,
            "timestamp_mismatch",
            left_reason,
            right_reason,
        )
    if body_error is not None:
        return CaptureResult(
            None,
            left_hand,
            right_hand,
            timestamp_after,
            f"body_error:{type(body_error).__name__}",
            left_reason,
            right_reason,
        )
    try:
        body = _finite_tracking_array(body_raw, BODY_SHAPE, field_name="body")
    except ValueError as exc:
        return CaptureResult(
            None,
            left_hand,
            right_hand,
            timestamp_after,
            str(exc),
            left_reason,
            right_reason,
        )
    return CaptureResult(
        body,
        left_hand,
        right_hand,
        timestamp_after,
        "ok",
        left_reason,
        right_reason,
    )


def _capture_atomic(
    sdk: ModuleType, attempts: int, *, include_body: bool = True
) -> CaptureResult:
    result = CaptureResult(None, None, None, 0, "no_attempt")
    for _ in range(attempts):
        result = _capture_once(sdk, include_body=include_body)
        if result.reason != "timestamp_mismatch":
            return result
    return result


def _positive_finite(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive and finite")
    return result


def _port(value: str) -> int:
    result = int(value)
    if not 1 <= result <= 65535:
        raise argparse.ArgumentTypeError("port must be in 1..65535")
    return result


def _positive_int(value: str) -> int:
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return result


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read controller-free PICO body and/or optical hands through "
            "XRoboToolkit and send timestamp-consistent UDP v1 frames"
        )
    )
    parser.add_argument("--host", default="127.0.0.1", help="UDP adapter host")
    parser.add_argument("--port", type=_port, default=5570, help="UDP adapter port")
    parser.add_argument(
        "--poll-hz",
        type=_positive_finite,
        default=120.0,
        help="maximum SDK polling frequency (default: 120)",
    )
    parser.add_argument(
        "--atomic-attempts",
        type=_positive_int,
        default=8,
        help="timestamp-consistency attempts per poll (default: 8)",
    )
    parser.add_argument(
        "--status-interval",
        type=_positive_finite,
        default=1.0,
        help="status log interval in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--sdk-module",
        default="xrobotoolkit_sdk",
        help="XRoboToolkit Python module name",
    )
    parser.add_argument(
        "--sdk-path",
        default="",
        help="optional directory containing the XRoboToolkit Python extension",
    )
    parser.add_argument(
        "--log-level",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        default="INFO",
    )
    parser.add_argument(
        "--hands-only",
        action="store_true",
        help=(
            "read only optical hands; do not call the body SDK API. The body "
            "slot in the UDP v1 envelope is zero padding and must be ignored"
        ),
    )
    return parser.parse_args(argv)


def _open_udp_peer(host: str, port: int) -> tuple[socket.socket, str]:
    addresses = socket.getaddrinfo(host, port, type=socket.SOCK_DGRAM)
    if not addresses:
        raise RuntimeError(f"cannot resolve UDP destination {host}:{port}")
    family, socktype, protocol, _, sockaddr = addresses[0]
    udp_socket = socket.socket(family, socktype, protocol)
    udp_socket.connect(sockaddr)
    return udp_socket, f"{sockaddr[0]}:{sockaddr[1]}"


def _body_available(sdk: ModuleType) -> bool:
    probe = getattr(sdk, "is_body_data_available", None)
    if not callable(probe):
        return True
    try:
        return bool(probe())
    except Exception:
        return False


def _log_status(
    counters: Counters,
    *,
    elapsed_s: float,
    last_device_timestamp_ns: int,
    last_left_reason: str,
    last_right_reason: str,
) -> None:
    rate = counters.sent / elapsed_s if elapsed_s > 0.0 else 0.0
    LOGGER.info(
        "status sent=%d rate=%.1fHz polls=%d unavailable=%d "
        "timestamp_mismatch=%d duplicate=%d body_rejected=%d "
        "left_missing=%d right_missing=%d send_errors=%d device_timestamp_ns=%d "
        "left=%s right=%s",
        counters.sent,
        rate,
        counters.polls,
        counters.unavailable,
        counters.timestamp_mismatch,
        counters.duplicate_timestamp,
        counters.body_rejected,
        counters.left_missing,
        counters.right_missing,
        counters.send_errors,
        last_device_timestamp_ns,
        last_left_reason,
        last_right_reason,
    )


def run(args: argparse.Namespace, stop_event: threading.Event) -> int:
    if args.sdk_path:
        sys.path.insert(0, str(Path(args.sdk_path).expanduser().resolve()))
    try:
        sdk = importlib.import_module(args.sdk_module)
    except ImportError as exc:
        raise RuntimeError(
            f"cannot import XRoboToolkit module {args.sdk_module!r}; "
            "run this script with the project's Python 3.10 teleop environment "
            "or provide --sdk-path"
        ) from exc

    udp_socket, destination = _open_udp_peer(args.host, args.port)
    counters = Counters()
    sequence = 0
    last_device_timestamp_ns = -1
    last_left_reason = "not_read"
    last_right_reason = "not_read"
    period_s = 1.0 / args.poll_hz
    started = time.monotonic()
    next_status = started + args.status_interval
    initialized = False

    LOGGER.info(
        "starting mode=%s destination=%s poll_hz=%.1f atomic_attempts=%d "
        "sdk=%s python=%s",
        "hands-only" if args.hands_only else "body+hands",
        destination,
        args.poll_hz,
        args.atomic_attempts,
        args.sdk_module,
        sys.version.split()[0],
    )
    try:
        sdk.init()
        initialized = True
        while not stop_event.is_set():
            loop_started = time.monotonic()
            counters.polls += 1
            if not args.hands_only and not _body_available(sdk):
                counters.unavailable += 1
            else:
                result = _capture_atomic(
                    sdk,
                    args.atomic_attempts,
                    include_body=not args.hands_only,
                )
                last_left_reason = result.left_reason
                last_right_reason = result.right_reason
                if result.reason == "timestamp_mismatch":
                    counters.timestamp_mismatch += 1
                elif result.body is None:
                    counters.body_rejected += 1
                elif result.device_timestamp_ns <= last_device_timestamp_ns:
                    counters.duplicate_timestamp += 1
                else:
                    if result.left_hand is None:
                        counters.left_missing += 1
                    if result.right_hand is None:
                        counters.right_missing += 1
                    frame = PicoUdpFrame(
                        sequence=sequence,
                        device_timestamp_ns=result.device_timestamp_ns,
                        source_timestamp_ns=time.monotonic_ns(),
                        body=result.body,
                        left_hand=result.left_hand,
                        right_hand=result.right_hand,
                    )
                    try:
                        datagram = encode_frame(frame)
                        sent_bytes = udp_socket.send(datagram)
                        if sent_bytes != len(datagram):
                            raise OSError(
                                f"partial UDP send: {sent_bytes}/{len(datagram)} bytes"
                            )
                    except OSError as exc:
                        counters.send_errors += 1
                        LOGGER.warning("UDP send failed: %s", exc)
                    else:
                        counters.sent += 1
                        sequence += 1
                        last_device_timestamp_ns = result.device_timestamp_ns

            now = time.monotonic()
            if now >= next_status:
                _log_status(
                    counters,
                    elapsed_s=now - started,
                    last_device_timestamp_ns=max(0, last_device_timestamp_ns),
                    last_left_reason=last_left_reason,
                    last_right_reason=last_right_reason,
                )
                next_status = now + args.status_interval
            stop_event.wait(max(0.0, period_s - (time.monotonic() - loop_started)))
    finally:
        if initialized:
            try:
                sdk.close()
            except Exception as exc:
                LOGGER.warning("XRoboToolkit close failed: %s", exc)
        udp_socket.close()

    elapsed = max(time.monotonic() - started, 1e-9)
    _log_status(
        counters,
        elapsed_s=elapsed,
        last_device_timestamp_ns=max(0, last_device_timestamp_ns),
        last_left_reason=last_left_reason,
        last_right_reason=last_right_reason,
    )
    LOGGER.info("stopped cleanly")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    stop_event = threading.Event()

    def request_stop(signum: int, _frame: Any) -> None:
        del signum
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        return run(args, stop_event)
    except KeyboardInterrupt:
        stop_event.set()
        return 0
    except Exception as exc:
        LOGGER.error("fatal: %s", exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
