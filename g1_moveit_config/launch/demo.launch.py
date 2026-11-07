# Copyright 2026 Yoshiki Obinata
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Yoshiki Obinata nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_spawn_controllers_launch,
    generate_static_virtual_joint_tfs_launch,
)

# Maps hand_type arg -> per-variant config filenames
_MODEL_CONFIG = {
    "no_hand": {
        "urdf_xacro": "g1_no_hand.urdf.xacro",
        "srdf": "g1_no_hand.srdf",
        "ros2_controllers": "ros2_controllers_no_hand.yaml",
    },
    "with_hand": {
        "urdf_xacro": "g1_29dof_with_hand.urdf.xacro",
        "srdf": "g1_29dof_with_hand.srdf",
        "ros2_controllers": "ros2_controllers.yaml",
    },
    "inspire_ftp": {
        "urdf_xacro": "g1_inspire_ftp.urdf.xacro",
        "srdf": "g1_inspire_ftp.srdf",
        "ros2_controllers": "ros2_controllers_inspire_ftp.yaml",
        "hand_controllers": ["right_hand_controller", "left_hand_controller"],
    },
    "inspire_dfq": {
        "urdf_xacro": "g1_inspire_dfq.urdf.xacro",
        "srdf": "g1_inspire_dfq.srdf",
        "ros2_controllers": "ros2_controllers_inspire_dfq.yaml",
        # Not used by MoveIt, but spawned like g1_bringup does on the real robot
        "hand_controllers": ["right_hand_controller", "left_hand_controller"],
    },
}


def _launch_setup(context, *args, **kwargs):
    hand_type = LaunchConfiguration("hand_type").perform(context)
    cfg = _MODEL_CONFIG[hand_type]

    moveit_config = (
        MoveItConfigsBuilder("g1_29dof", package_name="g1_moveit_config")
        .robot_description(file_path=f"config/{cfg['urdf_xacro']}")
        .robot_description_semantic(file_path=f"config/{cfg['srdf']}")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    g1_hw_share = Path(get_package_share_directory("g1_hardware"))
    ros2_controllers_yaml = str(g1_hw_share / f"config/{cfg['ros2_controllers']}")

    actions = []

    # Static TF for virtual joints
    actions.extend(generate_static_virtual_joint_tfs_launch(moveit_config).entities)

    # Robot state publisher
    actions.extend(generate_rsp_launch(moveit_config).entities)

    # MoveGroup node
    actions.extend(generate_move_group_launch(moveit_config).entities)

    # RViz (conditional on use_rviz)
    use_rviz = context.launch_configurations.get("use_rviz", "true").lower()
    if use_rviz not in ("false", "0"):
        actions.extend(generate_moveit_rviz_launch(moveit_config).entities)

    # ros2_control_node with the correct per-variant yaml
    actions.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[ros2_controllers_yaml],
            remappings=[
                ("/controller_manager/robot_description", "/robot_description"),
            ],
        )
    )

    # Spawn controllers
    actions.extend(generate_spawn_controllers_launch(moveit_config).entities)
    for name in cfg.get("hand_controllers", []):
        actions.append(
            Node(package="controller_manager", executable="spawner", arguments=[name])
        )

    # Warehouse DB (conditional on db)
    db = context.launch_configurations.get("db", "false").lower()
    if db not in ("false", "0"):
        from moveit_configs_utils.launches import generate_warehouse_db_launch

        actions.extend(generate_warehouse_db_launch(moveit_config).entities)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hand_type",
                default_value="no_hand",
                choices=["no_hand", "with_hand", "inspire_ftp", "inspire_dfq"],
                description=(
                    "G1 hand type: no_hand, with_hand (standard), "
                    "inspire_ftp (Inspire FTP), inspire_dfq (Inspire DFQ)"
                ),
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with MoveIt plugin",
            ),
            DeclareLaunchArgument(
                "db",
                default_value="false",
                description="Start warehouse database (MongoDB)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
