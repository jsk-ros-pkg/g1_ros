# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2026 Yoshiki Obinata
"""Top-level G1 nav2 bringup: localization + navigation.

  ros2 launch g1_navigation g1_navigation.launch.py
  ros2 launch g1_navigation g1_navigation.launch.py map:=/path/to/map.yaml
  ros2 launch g1_navigation g1_navigation.launch.py use_fast_lio:=false  # external odom
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = Path(get_package_share_directory("g1_navigation"))
    launch_dir = pkg / "launch"
    common = {
        "map": LaunchConfiguration("map"),
        "params_file": LaunchConfiguration("params_file"),
    }

    def include(name, extra=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / name)),
            launch_arguments={**(extra or {})}.items())

    return LaunchDescription([
        DeclareLaunchArgument("map", description="2D occupancy grid yaml (required)"),
        DeclareLaunchArgument("params_file",
                              default_value=str(pkg / "config" / "nav2_params.yaml")),
        DeclareLaunchArgument("use_fast_lio", default_value="true"),
        include("localization.launch.py", {
            **common, "use_fast_lio": LaunchConfiguration("use_fast_lio")}),
        include("navigation.launch.py", {"params_file": common["params_file"]}),
    ])
