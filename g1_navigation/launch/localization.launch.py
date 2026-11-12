# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2026 Yoshiki Obinata
"""G1 localization: FAST-LIO2 odometry + map_server + AMCL, with a
pointcloud_to_laserscan bridge (livox 3D -> /scan) for AMCL and costmaps."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = Path(get_package_share_directory("g1_navigation"))
    params = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    use_fast_lio = LaunchConfiguration("use_fast_lio")

    args = [
        DeclareLaunchArgument("map", description="2D occupancy grid yaml (required)"),
        DeclareLaunchArgument("params_file",
                              default_value=str(pkg / "config" / "nav2_params.yaml")),
        DeclareLaunchArgument("use_fast_lio", default_value="true"),
    ]

    p2l = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan")],
        parameters=[{
            "target_frame": "base_link",
            "transform_tolerance": 0.05,
            "min_height": -0.3,
            "max_height": 1.5,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0087,
            "range_min": 0.2,
            "range_max": 40.0,
            "use_inf": True,
        }],
    )

    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen", parameters=[params, {"yaml_filename": map_yaml}])
    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl",
        output="screen", parameters=[params])
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_localization", output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server", "amcl"]}])

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg / "launch" / "fast_lio.launch.py")),
        condition=IfCondition(use_fast_lio))

    return LaunchDescription(args + [p2l, map_server, amcl, lifecycle, fast_lio])
