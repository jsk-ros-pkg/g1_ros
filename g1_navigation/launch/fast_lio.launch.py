# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2026 Yoshiki Obinata
"""FAST-LIO2 odometry (odom -> base_link) for Livox mid360, plus the static
base_link -> pelvis link that bolts the robot TF (rooted at pelvis) under it.

Assumes the torso/waist is held fixed during navigation. FAST-LIO's base_link
sits at the IMU, so set base_link->pelvis from (torso neutral):
  ros2 run tf2_ros tf2_echo imu_in_torso pelvis
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = Path(get_package_share_directory("g1_navigation"))
    cfg = LaunchConfiguration("fast_lio_config")

    args = [DeclareLaunchArgument(
        "fast_lio_config",
        default_value=str(pkg / "config" / "fast_lio_mid360.yaml"))]
    # base_link -> pelvis placeholder (identity); override to the measured value.
    for a, d in (("x", "0.0"), ("y", "0.0"), ("z", "0.0"),
                 ("yaw", "0.0"), ("pitch", "0.0"), ("roll", "0.0")):
        args.append(DeclareLaunchArgument(f"base_to_pelvis_{a}", default_value=d))

    def lc(n):
        return LaunchConfiguration(f"base_to_pelvis_{n}")

    return LaunchDescription(args + [
        Node(package="fast_lio", executable="fastlio_mapping", name="fast_lio",
             output="screen", parameters=[cfg]),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_link_to_pelvis",
            arguments=[
                "--frame-id", "base_link", "--child-frame-id", "pelvis",
                "--x", lc("x"), "--y", lc("y"), "--z", lc("z"),
                "--yaw", lc("yaw"), "--pitch", lc("pitch"), "--roll", lc("roll"),
            ]),
    ])
