#!/usr/bin/env python3
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

"""Publish a validated 16 kHz mono PCM WAV file as audio_common AudioData."""

import argparse
import time
import wave

from audio_common_msgs.msg import AudioData, AudioInfo
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.utilities import remove_ros_args


def validate_wave(audio):
    """Validate the format and complete payload before sending any audio."""
    if (
        audio.getcomptype() != "NONE"
        or audio.getframerate() != 16000
        or audio.getnchannels() != 1
        or audio.getsampwidth() != 2
    ):
        raise ValueError("Expected uncompressed 16000 Hz mono 16-bit PCM WAV")
    expected = audio.getnframes() * 2
    if expected == 0:
        raise ValueError("WAV contains no audio frames")
    actual = 0
    while data := audio.readframes(1600):
        actual += len(data)
    if actual != expected:
        raise ValueError("WAV contains truncated or incomplete PCM samples")
    audio.rewind()


def main(args=None):
    """Send PCM samples in real time after waiting for a subscriber."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", help="Local 16000 Hz mono 16-bit PCM WAV file")
    cli = parser.parse_args(remove_ros_args(args=args)[1:])
    rclpy.init(args=args)
    node = Node("g1_audio_file_publisher")
    try:
        with wave.open(cli.file, "rb") as audio:
            validate_wave(audio)
            publisher = node.create_publisher(AudioData, "audio", 10)
            info_publisher = node.create_publisher(
                AudioInfo,
                "audio_info",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            deadline = time.monotonic() + 10.0
            while publisher.get_subscription_count() == 0:
                if not rclpy.ok() or time.monotonic() >= deadline:
                    raise RuntimeError("No audio subscriber found within 10 seconds")
                rclpy.spin_once(node, timeout_sec=0.1)
            info_publisher.publish(
                AudioInfo(
                    channels=1,
                    sample_rate=16000,
                    sample_format="S16LE",
                    bitrate=256000,
                    coding_format="wave",
                )
            )
            if not info_publisher.wait_for_all_acked(Duration(seconds=2)):
                raise RuntimeError("Timed out sending audio metadata")
            deadline = time.monotonic()
            while rclpy.ok() and (data := audio.readframes(1600)):
                publisher.publish(AudioData(data=data))
                deadline = max(deadline + len(data) / 32000, time.monotonic())
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(
                        node, timeout_sec=max(0.0, deadline - time.monotonic())
                    )
            if not publisher.wait_for_all_acked(Duration(seconds=2)):
                raise RuntimeError("Timed out sending audio data")
            node.get_logger().info("Audio sent; robot playback may still be finishing")
    except (OSError, EOFError, wave.Error, ValueError, RuntimeError) as error:
        node.get_logger().error(str(error))
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
