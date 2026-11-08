"""Launch only PICO optical-hand control; body control stays external/official."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("hand_zmq_endpoint", default_value="tcp://127.0.0.1:5560"),
            DeclareLaunchArgument("publish_rate", default_value="50.0"),
            DeclareLaunchArgument("source_timeout", default_value="0.5"),
            DeclareLaunchArgument("joint_state_timeout", default_value="0.5"),
            DeclareLaunchArgument("trajectory_duration", default_value="0.04"),
            DeclareLaunchArgument("velocity_margin", default_value="0.8"),
            DeclareLaunchArgument("calibration_file", default_value=""),
            DeclareLaunchArgument(
                "require_hand_state", default_value="true", choices=["true", "false"]
            ),
            Node(
                package="g1_pico_teleop",
                executable="pico_hand_adapter",
                name="pico_hand_adapter",
                output="screen",
                parameters=[
                    {
                        "hand_zmq_endpoint": LaunchConfiguration("hand_zmq_endpoint"),
                        "publish_rate": LaunchConfiguration("publish_rate"),
                        "source_timeout": LaunchConfiguration("source_timeout"),
                        "joint_state_timeout": LaunchConfiguration("joint_state_timeout"),
                        "trajectory_duration": LaunchConfiguration("trajectory_duration"),
                        "velocity_margin": LaunchConfiguration("velocity_margin"),
                        "require_hand_state": LaunchConfiguration("require_hand_state"),
                        "calibration_file": LaunchConfiguration("calibration_file"),
                    }
                ],
            ),
        ]
    )
