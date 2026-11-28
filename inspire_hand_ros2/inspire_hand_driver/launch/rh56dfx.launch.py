"""Launch one physical RH56DFX hand in its own namespace."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
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
            "exit_on_disconnect",
        )
    }
    mappings["action_prefix"] = namespace
    description = xacro.process_file(
        str(share / "urdf/rh56dfx.urdf.xacro"), mappings=mappings
    ).toxml()
    prefix = "L_" if mappings["side"] == "left" else "R_"
    context.launch_configurations["rh56dfx_prefix"] = prefix
    controller_params = ParameterFile(
        share / "config/controllers.yaml", allow_substs=True
    )
    controllers = str(controller_params.evaluate(context))
    tf_topics = [("/tf", "tf"), ("/tf_static", "tf_static")]
    manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[controllers],
        respawn=mappings["exit_on_disconnect"] == "true",
        respawn_delay=3.0,
    )

    def spawn_controllers(event, context):
        if context.is_shutdown:
            return []
        spawner = Node(
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
        )
        # A manager can exit before this spawner finishes waiting for services.
        cleanup = OnProcessExit(
            target_action=manager,
            on_exit=[
                EmitEvent(
                    event=ShutdownProcess(process_matcher=matches_action(spawner))
                )
            ],
            handle_once=True,
        )
        return [RegisterEventHandler(cleanup), spawner]

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[{"robot_description": description}],
            remappings=tf_topics,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            namespace=namespace,
            arguments=[
                "-d",
                str(share / "config/rh56dfx.rviz"),
                "-f",
                prefix + "hand_base_link",
            ],
            remappings=tf_topics,
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=manager, on_start=spawn_controllers)
        ),
        RegisterEventHandler(
            OnShutdown(on_shutdown=lambda event, ctx: controller_params.cleanup())
        ),
        manager,
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
                "exit_on_disconnect",
                default_value="true",
                choices=["true", "false"],
                description="Exit and respawn the standalone driver if its serial hand is absent or unplugged",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="false", choices=["true", "false"]
            ),
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
