# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2026 Yoshiki Obinata
"""Nav2 navigation servers for G1 (controller/planner/behaviors/bt).
controller_server publishes /cmd_vel (Twist) to the gear_sonic bridge."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = Path(get_package_share_directory("g1_navigation"))
    params = LaunchConfiguration("params_file")

    nodes = ["controller_server", "planner_server", "behavior_server",
             "bt_navigator"]

    servers = [
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen", parameters=[params]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=[params]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=[params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=[params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"autostart": True, "node_names": nodes}]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("params_file",
                              default_value=str(pkg / "config" / "nav2_params.yaml")),
    ] + servers)
