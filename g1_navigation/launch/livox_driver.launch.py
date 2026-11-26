# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2026 Yoshiki Obinata
"""Livox mid360 driver publishing CustomMsg on /livox/lidar_custom (+ imu).

FAST-LIO's Livox path (lidar_type=1) uses livox_ros_driver2::msg::CustomMsg,
whose per-point timestamps give better odometry than PointCloud2. Dedicated
topics let this coexist with unitree's PointCloud2 /livox/lidar (scan bridge).
Edit the host/lidar IPs in config/mid360.json for your robot.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = Path(get_package_share_directory("g1_navigation"))
    config = LaunchConfiguration("mid360_config")

    return LaunchDescription([
        DeclareLaunchArgument("mid360_config",
                              default_value=str(pkg / "config" / "mid360.json")),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                "xfer_format": 1,        # 1 = livox CustomMsg
                "multi_topic": 0,
                "data_src": 0,           # 0 = lidar
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "lvx_file_path": "",
                "user_config_path": config,
                "cmdline_input_bd_code": "livox0000000001",
            }],
            # Dedicated topics so this coexists with unitree's PointCloud2
            # /livox/lidar and /livox/imu.
            remappings=[("livox/points", "/livox/lidar_custom"),
                        ("livox/imu", "/livox/imu_custom")],
        ),
    ])
