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

"""
Play back a rosbag recorded by rosbag_record.launch.py.

The compressed sensor streams replay as-is; consumers (RViz included)
subscribe through image_transport / point_cloud_transport and need no
republisher. The exception: bags recorded with record_points:=false hold no
D435i cloud, so it is rebuilt here from the depth and color images, which
depth_image_proc only accepts raw.

Do not run this against the live robot: the bag republishes /joint_states,
/tf and /robot_description, which would fight the real robot_state_publisher.

Usage:
  ros2 launch g1_bringup rosbag_play.launch.py bag:=/home/unitree/bags/pick_demo

  # Half speed, looping, starting 10 s in
  ros2 launch g1_bringup rosbag_play.launch.py bag:=./pick_demo \
      rate:=0.5 loop:=true start_offset:=10.0

  # Drive nodes off the bag clock (they need use_sim_time:=true themselves)
  ros2 launch g1_bringup rosbag_play.launch.py bag:=./pick_demo clock:=true

SPACE pauses/resumes; Ctrl-C stops. Headless runs are controlled through
the /rosbag2_player services instead.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode
import yaml

_D435I_BASE = "/head_camera/d435"
_D435I_COLOR_TOPIC = f"{_D435I_BASE}/color/image_raw"
_D435I_DEPTH_TOPIC = f"{_D435I_BASE}/depth/image_rect_raw"
_D435I_POINTS_TOPIC = f"{_D435I_BASE}/depth/color/points"


def _bag_topics(bag):
    """
    Return the set of topic names stored in the bag.

    None when the metadata cannot be read (e.g. a bare .mcap file was passed);
    callers then assume every stream might be present.
    """
    metadata = Path(bag) / "metadata.yaml"
    if not metadata.is_file():
        return None
    try:
        info = yaml.safe_load(metadata.read_text())
        topics = info["rosbag2_bagfile_information"]["topics_with_message_count"]
        return {entry["topic_metadata"]["name"] for entry in topics}
    except (AttributeError, KeyError, TypeError, yaml.YAMLError):
        return None


def _image_decompressor(name, raw_topic, transport):
    """
    Decode an image_transport-compressed topic back to sensor_msgs/Image.

    Only feeds the reconstruction below: depth_image_proc needs the raw
    images and a bag holds only the compressed ones. These decode at the
    full bag rate (measured 183 of 183 frames on both streams).
    """
    return Node(
        package="image_transport",
        executable="republish",
        name=name,
        parameters=[{"in_transport": transport, "out_transport": "raw"}],
        remappings=[
            (f"in/{transport}", f"{raw_topic}/{transport}"),
            ("out", raw_topic),
        ],
    )


def _points_reconstruction():
    """
    Rebuild an approximate D435i colored cloud from the depth and color images.

    For bags recorded with record_points:=false. Unlike the cloud the robot
    published it is limited to the color FOV (measured 119k of 242k points)
    and stamped d435_color_optical_frame instead of d435_depth_optical_frame;
    the recorded static TF between the two keeps the geometry consistent.

    One shared container with intra-process comms keeps the intermediate
    registered depth image from being serialized.
    """
    color_info = f"{_D435I_BASE}/color/camera_info"
    registered = f"{_D435I_BASE}/depth_registered/image_rect"
    return ComposableNodeContainer(
        name="d435_points_reconstruction",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::RegisterNode",
                name="d435_depth_register",
                remappings=[
                    ("rgb/camera_info", color_info),
                    ("depth/camera_info", f"{_D435I_BASE}/depth/camera_info"),
                    ("depth/image_rect", _D435I_DEPTH_TOPIC),
                    (
                        "depth_registered/camera_info",
                        f"{_D435I_BASE}/depth_registered/camera_info",
                    ),
                    ("depth_registered/image_rect", registered),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="d435_points_xyzrgb",
                remappings=[
                    ("rgb/camera_info", color_info),
                    ("rgb/image_rect_color", _D435I_COLOR_TOPIC),
                    ("depth_registered/image_rect", registered),
                    ("points", _D435I_POINTS_TOPIC),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )


def _launch_setup(context, *args, **kwargs):
    def flag(name):
        return LaunchConfiguration(name).perform(context).lower() in ("true", "1")

    def value(name):
        return LaunchConfiguration(name).perform(context)

    bag = value("bag")
    if not bag:
        raise RuntimeError("bag:=<path to the bag directory> is required")
    if not Path(bag).exists():
        raise RuntimeError(f"bag not found: {bag}")

    clock = flag("clock")

    cmd = ["ros2", "bag", "play", bag, "--rate", value("rate")]

    start_offset = float(value("start_offset"))
    if start_offset > 0.0:
        cmd += ["--start-offset", str(start_offset)]

    if flag("loop"):
        cmd.append("--loop")

    if flag("start_paused"):
        cmd.append("--start-paused")

    if clock:
        cmd += ["--clock", value("clock_rate")]

    topics = value("topics").split()
    if topics:
        cmd += ["--topics"] + topics

    qos_overrides = value("qos_overrides")
    if qos_overrides:
        cmd += ["--qos-profile-overrides-path", qos_overrides]

    actions = []

    # No recorded cloud in the bag -> rebuild it, see _points_reconstruction().
    bag_topics = _bag_topics(bag)
    if bag_topics is not None:
        have_images = {
            f"{_D435I_COLOR_TOPIC}/compressed",
            f"{_D435I_DEPTH_TOPIC}/compressedDepth",
        } <= bag_topics
        if have_images and f"{_D435I_POINTS_TOPIC}/zstd" not in bag_topics:
            actions += [
                _image_decompressor(
                    "d435_color_decompressor", _D435I_COLOR_TOPIC, "compressed"
                ),
                _image_decompressor(
                    "d435_depth_decompressor", _D435I_DEPTH_TOPIC, "compressedDepth"
                ),
                _points_reconstruction(),
            ]
            actions.append(
                LogInfo(
                    msg=(
                        "no recorded cloud; rebuilding "
                        + _D435I_POINTS_TOPIC
                        + " from the depth and color images (approximate: "
                        + "color FOV only, d435_color_optical_frame)"
                    )
                )
            )

    # Only reaches the reconstruction container above; other nodes need
    # their own use_sim_time:=true to follow the bag clock.
    if clock:
        actions.insert(0, SetParameter(name="use_sim_time", value=True))

    # launch's pipe as stdin would disable the player's keyboard controls;
    # hand it the controlling terminal when there is one.
    try:
        with open("/dev/tty"):
            cmd = ["bash", "-c", 'exec "$0" "$@" < /dev/tty'] + cmd
    except OSError:
        pass

    actions.append(ExecuteProcess(cmd=cmd, output="screen"))
    return actions


def generate_launch_description():
    default_qos_overrides = str(
        Path(get_package_share_directory("g1_bringup")) / "config/rosbag_play_qos.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag",
                description="Path to the bag directory to play",
            ),
            DeclareLaunchArgument(
                "rate", default_value="1.0", description="Playback rate multiplier"
            ),
            DeclareLaunchArgument(
                "loop",
                default_value="false",
                choices=["true", "false"],
                description="Restart the bag when it reaches the end",
            ),
            DeclareLaunchArgument(
                "start_offset",
                default_value="0.0",
                description="Skip this many seconds from the start of the bag",
            ),
            DeclareLaunchArgument(
                "start_paused",
                default_value="false",
                choices=["true", "false"],
                description="Start paused; press SPACE to begin",
            ),
            DeclareLaunchArgument(
                "clock",
                default_value="false",
                choices=["true", "false"],
                description="Publish /clock from the bag timestamps",
            ),
            DeclareLaunchArgument(
                "clock_rate",
                default_value="100",
                description="/clock publish rate in Hz (with clock:=true)",
            ),
            DeclareLaunchArgument(
                "topics",
                default_value="",
                description="Space-separated subset of topics to play; empty plays all",
            ),
            DeclareLaunchArgument(
                "qos_overrides",
                default_value=default_qos_overrides,
                description=(
                    "rosbag2 QoS override file. The default replays the sensor "
                    "topics as reliable, which any subscriber accepts; "
                    "best_effort as recorded would be refused by a "
                    "default-QoS (reliable) subscriber"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
