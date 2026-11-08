"""Launch only PICO optical-hand control; body control stays external/official."""

from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_input(context):
    if (LaunchConfiguration("hand_zmq_endpoint").perform(context).strip()
            and LaunchConfiguration("start_xrt_source").perform(context).lower() == "true"):
        raise RuntimeError("Choose robot hand_zmq_endpoint or local start_xrt_source, not both")
    return []


def generate_launch_description():
    prefix = Path(get_package_prefix("g1_pico_teleop"))
    source_script = prefix / "lib" / "g1_pico_teleop" / "pico_xrt_source.py"

    listen_address = LaunchConfiguration("listen_address")
    listen_port = LaunchConfiguration("listen_port")
    source_destination_host = LaunchConfiguration("source_destination_host")
    xrt_python = LaunchConfiguration("xrt_python_executable")

    return LaunchDescription(
        [
            DeclareLaunchArgument("listen_address", default_value="127.0.0.1"),
            DeclareLaunchArgument("listen_port", default_value="5570"),
            DeclareLaunchArgument("hand_zmq_endpoint", default_value=""),
            DeclareLaunchArgument("source_destination_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("publish_rate", default_value="50.0"),
            DeclareLaunchArgument("source_timeout", default_value="0.5"),
            DeclareLaunchArgument("joint_state_timeout", default_value="0.5"),
            DeclareLaunchArgument("trajectory_duration", default_value="0.04"),
            DeclareLaunchArgument("velocity_margin", default_value="0.8"),
            DeclareLaunchArgument("calibration_file", default_value=""),
            DeclareLaunchArgument(
                "require_hand_state", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "start_xrt_source", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("xrt_python_executable", default_value="python3"),
            DeclareLaunchArgument("xrt_sdk_path", default_value=""),
            OpaqueFunction(function=_validate_input),
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration("start_xrt_source")),
                cmd=[
                    xrt_python,
                    str(source_script),
                    "--hands-only",
                    "--host",
                    source_destination_host,
                    "--port",
                    listen_port,
                    "--sdk-path",
                    LaunchConfiguration("xrt_sdk_path"),
                ],
                output="screen",
            ),
            Node(
                package="g1_pico_teleop",
                executable="pico_hand_adapter",
                name="pico_hand_adapter",
                output="screen",
                parameters=[
                    {
                        "listen_address": listen_address,
                        "listen_port": listen_port,
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
