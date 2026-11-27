"""Launch one physical RH56DFX hand in its own namespace."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _setup(context):
    share = Path(get_package_share_directory("inspire_hand_driver"))
    namespace = "/" + LaunchConfiguration("namespace").perform(context).strip("/")
    mappings = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            "side",
            "device",
            "baudrate",
            "hand_id",
            "close_hand_on_deactivate",
        )
    }
    mappings["action_prefix"] = namespace
    description = xacro.process_file(
        str(share / "urdf/rh56dfx.urdf.xacro"), mappings=mappings
    ).toxml()
    controllers = str(share / "config/controllers.yaml")
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[{"robot_description": description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace,
            parameters=[controllers],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=[
                "joint_state_broadcaster",
                "position_joint_trajectory_controller",
                "--param-file",
                controllers,
                "--controller-ros-args",
                "--remap /joint_states:=joint_states",
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "baudrate", default_value="115200", choices=["115200", "57600", "19200"]
            ),
            DeclareLaunchArgument(
                "hand_id", default_value="1", description="Serial device ID (1-254)"
            ),
            DeclareLaunchArgument("namespace", default_value="inspire_hand"),
            DeclareLaunchArgument(
                "side", default_value="right", choices=["right", "left"]
            ),
            DeclareLaunchArgument(
                "device", description="Serial port connected to this hand"
            ),
            DeclareLaunchArgument(
                "close_hand_on_deactivate",
                default_value="true",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=_setup),
        ]
    )
