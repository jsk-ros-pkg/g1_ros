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
G1 real-hardware ROS2 driver launch.

Starts:
  - robot_state_publisher    (URDF with g1_hardware plugin)
  - ros2_control_node        (talks to robot via DDS)
  - loco_cmd_adapter         (accepting locomotion mode, cmd_vel;
                              skipped with use_gear_sonic:=true — SONIC owns /cmd_vel)
  - joint_state_broadcaster  (infrastructure: required for TF / RViz)
  - gear_sonic_interface     (optional: SONIC deploy bridge, use_gear_sonic:=true)

Application-specific controller spawning (upper_body_controller,
hand controllers, etc.) is intentionally left to the bringup layer.

Usage:
  ros2 launch g1_hardware hardware.launch.py network_interface:=eth0
  # With the gear_sonic (SONIC) locomotion bridge:
  #   zmq_host is the XPUB *bind* address. The default 127.0.0.1 only accepts
  #   a deploy stack on the same machine (sim). For the real robot, expose it
  #   with zmq_host:=0.0.0.0 so the on-robot deploy stack can connect.
  ros2 launch g1_hardware hardware.launch.py network_interface:=lo use_gear_sonic:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import xacro

_HARDWARE_CONFIG = {
    "no_hand": {
        "urdf_xacro": "g1_no_hand_real.urdf.xacro",
        "ros2_controllers": "ros2_controllers_no_hand.yaml",
    },
    "inspire_dfq": {
        "urdf_xacro": "g1_inspire_dfq_real.urdf.xacro",
        "ros2_controllers": "ros2_controllers_inspire_dfq.yaml",
    },
}


def _launch_setup(context, *args, **kwargs):
    hand_type = LaunchConfiguration("hand_type").perform(context)

    if hand_type not in _HARDWARE_CONFIG:
        raise RuntimeError(
            f"hand_type '{hand_type}' has no real-hardware config yet. "
            f"Available: {list(_HARDWARE_CONFIG)}"
        )

    cfg = _HARDWARE_CONFIG[hand_type]

    network_interface = LaunchConfiguration("network_interface").perform(context)
    use_gear_sonic = LaunchConfiguration("use_gear_sonic").perform(context).lower()
    # With gear_sonic the URDF gains the virtual pelvis_height_joint so that
    # MoveIt (and RViz, which reads /robot_description) can plan squatting.
    # ros2_control ignores the extra passive joint.
    use_pelvis_lift = "true" if use_gear_sonic == "true" else "false"
    kp = LaunchConfiguration("kp").perform(context)
    kd = LaunchConfiguration("kd").perform(context)
    waist_kp = LaunchConfiguration("waist_kp").perform(context)
    waist_kd = LaunchConfiguration("waist_kd").perform(context)
    inspire_command_topic = LaunchConfiguration("inspire_command_topic").perform(
        context
    )
    inspire_state_topic = LaunchConfiguration("inspire_state_topic").perform(context)
    inspire_state_timeout_sec = LaunchConfiguration(
        "inspire_state_timeout_sec"
    ).perform(context)
    close_hand_on_deactivate = LaunchConfiguration("close_hand_on_deactivate").perform(
        context
    )

    g1_hw_share = Path(get_package_share_directory("g1_hardware"))

    urdf_path = g1_hw_share / f"config/{cfg['urdf_xacro']}"
    robot_description = xacro.process_file(
        str(urdf_path),
        mappings={
            "network_interface": network_interface,
            "kp": kp,
            "kd": kd,
            "waist_kp": waist_kp,
            "waist_kd": waist_kd,
            "inspire_command_topic": inspire_command_topic,
            "inspire_state_topic": inspire_state_topic,
            "inspire_state_timeout_sec": inspire_state_timeout_sec,
            "close_hand_on_deactivate": close_hand_on_deactivate,
            "use_pelvis_lift": use_pelvis_lift,
        },
    ).toxml()

    controllers_yaml = str(g1_hw_share / f"config/{cfg['ros2_controllers']}")

    composable_nodes = [
        ComposableNode(
            package="robot_state_publisher",
            plugin="robot_state_publisher::RobotStatePublisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        ComposableNode(
            package="g1_hardware",
            plugin="g1_hardware::G1LivoxInterfaceNode",
            name="g1_livox_interface",
            parameters=[{"network_interface": network_interface}],
        ),
    ]

    if use_gear_sonic != "true":
        composable_nodes.append(
            ComposableNode(
                package="g1_hardware",
                plugin="loco_cmd_adapter::LocoCmdAdapterNode",
                name="loco_cmd_adapter",
                parameters=[{"network_interface": network_interface}],
            )
        )

    if use_gear_sonic == "true":
        zmq_host = LaunchConfiguration("zmq_host").perform(context)
        composable_nodes.append(
            ComposableNode(
                package="g1_hardware",
                plugin="g1_hardware::GearSonicInterface",
                name="gear_sonic_interface",
                parameters=[
                    {
                        "zmq_host": zmq_host,
                        "left_wrist_compliance": float(
                            LaunchConfiguration("left_wrist_compliance").perform(
                                context
                            )
                        ),
                        "right_wrist_compliance": float(
                            LaunchConfiguration("right_wrist_compliance").perform(
                                context
                            )
                        ),
                        "head_compliance": float(
                            LaunchConfiguration("head_compliance").perform(context)
                        ),
                    }
                ],
            )
        )
        # FollowJointTrajectory -> VR 3-point adapter (MoveIt-compatible)
        composable_nodes.append(
            ComposableNode(
                package="g1_hardware",
                plugin="g1_hardware::GearSonicController",
                name="gear_sonic_controller",
            )
        )

    audio_nodes = []
    if LaunchConfiguration("use_audio").perform(context).lower() == "true":
        audio_nodes.append(
            Node(
                package="g1_hardware",
                executable="g1_audio_capture_node",
                parameters=[{"network_interface": network_interface}],
            )
        )
        audio_nodes.append(
            Node(
                package="g1_hardware",
                executable="g1_audio_interface_node",
                parameters=[{"network_interface": network_interface}],
            )
        )

    return [
        *audio_nodes,
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description},
                controllers_yaml,
            ],
        ),
        ComposableNodeContainer(
            name="g1_hardware_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=composable_nodes,
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hand_type",
                default_value="no_hand",
                choices=list(_HARDWARE_CONFIG),
                description="G1 hand configuration",
            ),
            DeclareLaunchArgument(
                "network_interface",
                description="Network interface connected to the G1 robot (e.g. 'eth0')",
            ),
            DeclareLaunchArgument(
                "kp", default_value="60.0", description="Arm position gain"
            ),
            DeclareLaunchArgument(
                "kd", default_value="1.5", description="Arm velocity (damping) gain"
            ),
            DeclareLaunchArgument(
                "waist_kp", default_value="200.0", description="Waist position gain"
            ),
            DeclareLaunchArgument(
                "waist_kd",
                default_value="2.0",
                description="Waist velocity (damping) gain",
            ),
            DeclareLaunchArgument(
                "inspire_command_topic",
                default_value="rt/inspire/cmd",
                description="DDS topic for RH56DFX commands",
            ),
            DeclareLaunchArgument(
                "inspire_state_topic",
                default_value="rt/inspire/state",
                description="DDS topic for RH56DFX state",
            ),
            DeclareLaunchArgument(
                "inspire_state_timeout_sec",
                default_value="3.0",
                description="Seconds to wait for the first RH56DFX state message",
            ),
            DeclareLaunchArgument(
                "close_hand_on_deactivate",
                default_value="true",
                choices=["true", "false"],
                description="Close RH56DFX hands when the hardware interface deactivates",
            ),
            DeclareLaunchArgument(
                "use_audio",
                default_value="true",
                choices=["true", "false"],
                description="Start G1 audio playback, microphone capture, TTS and ASR",
            ),
            DeclareLaunchArgument(
                "use_gear_sonic",
                default_value="false",
                choices=["true", "false"],
                description="Start the gear_sonic (SONIC) locomotion bridge",
            ),
            DeclareLaunchArgument(
                "zmq_host",
                default_value="127.0.0.1",
                description=(
                    "gear_sonic_interface XPUB bind address (only used with "
                    "use_gear_sonic:=true)."
                ),
            ),
            DeclareLaunchArgument(
                "left_wrist_compliance",
                default_value="0.0",
                description=(
                    "SONIC left wrist tracking compliance, 0.0-1.0; 0.0 = stiff "
                    "(exact tracking). Only used with use_gear_sonic:=true."
                ),
            ),
            DeclareLaunchArgument(
                "right_wrist_compliance",
                default_value="0.0",
                description=(
                    "SONIC right wrist tracking compliance, 0.0-1.0; 0.0 = stiff "
                    "(exact tracking). Only used with use_gear_sonic:=true."
                ),
            ),
            DeclareLaunchArgument(
                "head_compliance",
                default_value="0.0",
                description=(
                    "SONIC head tracking compliance, 0.0-1.0; 0.0 = stiff "
                    "(exact tracking). Only used with use_gear_sonic:=true."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
