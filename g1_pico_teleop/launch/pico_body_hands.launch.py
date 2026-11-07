"""Workstation adapters for an unchanged robot bringup with use_gear_sonic=true."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("g1_pico_teleop"))
    return LaunchDescription([
        DeclareLaunchArgument("body_zmq_endpoint", default_value="tcp://10.42.0.1:5558"),
        DeclareLaunchArgument("hand_zmq_endpoint", default_value="tcp://10.42.0.1:5560"),
        DeclareLaunchArgument("sonic_prefix", default_value="/gear_sonic_interface"),
        DeclareLaunchArgument("balance_duration", default_value="3.0"),
        Node(
            package="g1_pico_teleop", executable="pico_body_adapter",
            name="pico_body_adapter", output="screen",
            sigterm_timeout="10",
            parameters=[{
                "body_zmq_endpoint": LaunchConfiguration("body_zmq_endpoint"),
                "sonic_prefix": LaunchConfiguration("sonic_prefix"),
                "balance_duration": LaunchConfiguration("balance_duration"),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/pico_hands.launch.py")),
            launch_arguments={"hand_zmq_endpoint": LaunchConfiguration("hand_zmq_endpoint")}.items(),
        ),
    ])
