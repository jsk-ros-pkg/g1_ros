# Copyright 2026 Yoshiki Obinata
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Yoshiki Obinata nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Validate WAV format and truncation handling without a robot."""

import importlib.util
import io
from pathlib import Path
import struct
import subprocess
import sys
import time
import wave

from audio_common_msgs.msg import AudioData, AudioInfo
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

_MODULE_PATH = Path(__file__).parents[1] / "scripts" / "play_audio.py"
_SPEC = importlib.util.spec_from_file_location("play_audio", _MODULE_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


def make_wave(rate=16000, channels=1, width=2, frames=1601):
    """Create a PCM WAV fixture with a partial final transmission chunk."""
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as audio:
        audio.setnchannels(channels)
        audio.setsampwidth(width)
        audio.setframerate(rate)
        audio.writeframes(bytes(frames * channels * width))
    return buffer.getvalue()


def test_valid_wave_rewinds():
    """Validation preserves the complete payload for transmission."""
    with wave.open(io.BytesIO(make_wave()), "rb") as audio:
        _MODULE.validate_wave(audio)
        assert audio.tell() == 0
        assert len(audio.readframes(1601)) == 3202


@pytest.mark.parametrize(
    "kwargs",
    [{"rate": 48000}, {"channels": 2}, {"width": 1}, {"width": 3}, {"frames": 0}],
)
def test_reject_wrong_format(kwargs):
    """Reject formats that Unitree cannot play."""
    with wave.open(io.BytesIO(make_wave(**kwargs)), "rb") as audio:
        with pytest.raises(ValueError):
            _MODULE.validate_wave(audio)


@pytest.mark.parametrize("missing", [1, 2, 200])
def test_reject_truncated_data(missing):
    """Reject truncated samples before publishing the first chunk."""
    with wave.open(io.BytesIO(make_wave()[:-missing]), "rb") as audio:
        with pytest.raises(ValueError):
            _MODULE.validate_wave(audio)


def test_reject_odd_data_size():
    """Reject a data chunk ending halfway through a sample."""
    data = bytearray(make_wave())
    struct.pack_into("<I", data, 40, 3201)
    with wave.open(io.BytesIO(data), "rb") as audio:
        with pytest.raises(ValueError):
            _MODULE.validate_wave(audio)


def test_accept_extra_riff_chunk():
    """Do not assume every WAV file has a 44-byte header."""
    data = bytearray(make_wave())
    data[36:36] = b"JUNK" + struct.pack("<I", 3) + b"abc\0"
    struct.pack_into("<I", data, 4, len(data) - 8)
    with wave.open(io.BytesIO(data), "rb") as audio:
        _MODULE.validate_wave(audio)
        assert len(audio.readframes(1601)) == 3202


def test_publisher_sends_pcm_and_metadata(tmp_path):
    """Exercise the CLI over ROS without publishing to robot command topics."""
    path = tmp_path / "audio.wav"
    path.write_bytes(make_wave())
    context = Context()
    rclpy.init(context=context)
    node = Node("audio_receiver", namespace="/g1_audio_publisher_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    chunks = []
    metadata = []
    node.create_subscription(
        AudioData, "audio", lambda msg: chunks.append(bytes(msg.data)), 10
    )
    node.create_subscription(
        AudioInfo,
        "audio_info",
        metadata.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    process = subprocess.Popen(
        [
            sys.executable,
            str(_MODULE_PATH),
            str(path),
            "--ros-args",
            "-r",
            "__ns:=/g1_audio_publisher_test",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        deadline = time.monotonic() + 15
        while process.poll() is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        # Drain messages already acknowledged by DDS.
        for _ in range(5):
            executor.spin_once(timeout_sec=0.05)
        assert process.poll() is not None, "Publisher did not exit"
        output = process.communicate(timeout=1)[0]
        assert process.returncode == 0, output
        assert [len(chunk) for chunk in chunks] == [3200, 2]
        assert b"".join(chunks) == bytes(3202)
        assert metadata
        assert metadata[0].sample_rate == 16000
        assert metadata[0].sample_format == "S16LE"
        assert metadata[0].channels == 1
        assert metadata[0].coding_format == "wave"
        assert metadata[0].bitrate == 256000
    finally:
        if process.poll() is None:
            process.kill()
        process.communicate()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
