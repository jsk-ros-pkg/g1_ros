# Copyright 2026 JSK ROBOTICS LAB
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
#    * Neither the name of the JSK ROBOTICS LAB nor the names of its
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

"""
Human-robot interaction I/O that runs beside the G1 bringup.

Currently: stream the G1 internal USB microphone with audio_capture
(disable with use_mic:=false).

This is a stopgap until the built-in microphone receiver
(g1_audio_capture_node, jsk-ros-pkg/g1_ros#54) delivers real audio; it
publishes the same topics, format and QoS so consumers need no change:
  /microphone/audio          audio_common_msgs/AudioData
  /microphone/audio_info     audio_common_msgs/AudioInfo (transient local)
  /microphone/audio_stamped  audio_common_msgs/AudioDataStamped
as raw PCM (wave, S16LE / 16 kHz / mono, stamped with ROS time). Do not run
both at once, or the two streams are interleaved on the same topics.

device:=auto (default) picks the first USB sound card that has a capture PCM
(by sysfs, so it also works inside a container where /proc/asound is masked)
and opens it as plughw:CARD=<id>,DEV=<n>. The card *name* is used rather than
its index, which can change between boots. plughw lets ALSA convert from the
mic's native rate/channels. If no USB mic is found the launch fails, and
ros-usb-microphone.service retries until the mic is plugged in.

Usage:
  ros2 launch g1_hardware interaction.launch.py
  ros2 launch g1_hardware interaction.launch.py device:=plughw:CARD=Device,DEV=0
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_SYSFS_SOUND = Path("/sys/class/sound")


def _find_usb_capture_device():
    cards = sorted(_SYSFS_SOUND.glob("card[0-9]*"), key=lambda card: int(card.name[4:]))
    for card in cards:
        if "/usb" not in os.path.realpath(card / "device"):
            continue
        captures = sorted(
            card.glob(f"pcmC{card.name[4:]}D*c"),
            key=lambda pcm: int(pcm.name.split("D")[1][:-1]),
        )
        if not captures:
            continue
        card_id = (card / "id").read_text().strip()
        dev = captures[0].name.split("D")[1][:-1]
        return f"plughw:CARD={card_id},DEV={dev}"
    return None


def _launch_setup(context, *args, **kwargs):
    if LaunchConfiguration("use_mic").perform(context) != "true":
        return []
    device = LaunchConfiguration("device").perform(context)
    if device == "auto":
        device = _find_usb_capture_device()
        if device is None:
            raise RuntimeError(
                f"No USB capture device found under {_SYSFS_SOUND}. Is the mic "
                "connected and /dev/snd passed to the container? "
                "Set device:=<ALSA device> to choose one explicitly."
            )

    def lc(name):
        return LaunchConfiguration(name).perform(context)

    return [
        LogInfo(msg=f"Capturing audio from ALSA device '{device}'"),
        Node(
            package="audio_capture",
            executable="audio_capture_node",
            name="audio_capture",
            output="screen",
            parameters=[
                {
                    "src": "alsasrc",
                    "dst": "appsink",
                    "device": device,
                    "format": lc("format"),
                    "bitrate": int(lc("bitrate")),
                    "channels": int(lc("channels")),
                    "depth": int(lc("depth")),
                    "sample_rate": int(lc("sample_rate")),
                    "sample_format": lc("sample_format"),
                }
            ],
            remappings=[
                ("audio", "microphone/audio"),
                ("audio_info", "microphone/audio_info"),
                ("audio_stamped", "microphone/audio_stamped"),
            ],
            # Stop the whole launch (and the container) if capture dies, so
            # the systemd unit restarts it instead of leaving a silent topic.
            on_exit=Shutdown(reason="audio_capture_node exited"),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mic",
                default_value="true",
                choices=["true", "false"],
                description="Stream the USB microphone to /microphone/audio",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="auto",
                description=(
                    "ALSA capture device, e.g. plughw:CARD=Device,DEV=0 "
                    "(see `arecord -L`). 'auto' = first USB capture device"
                ),
            ),
            DeclareLaunchArgument(
                "format",
                default_value="wave",
                choices=["wave", "mp3"],
                description="Published encoding: raw PCM (wave) or mp3",
            ),
            DeclareLaunchArgument(
                "bitrate", default_value="128", description="mp3 bitrate [kbps]"
            ),
            DeclareLaunchArgument(
                "channels", default_value="1", description="Number of channels"
            ),
            DeclareLaunchArgument(
                "depth", default_value="16", description="Bits per sample"
            ),
            DeclareLaunchArgument(
                "sample_rate", default_value="16000", description="Sample rate [Hz]"
            ),
            DeclareLaunchArgument(
                "sample_format",
                default_value="S16LE",
                description="GStreamer raw audio sample format",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
