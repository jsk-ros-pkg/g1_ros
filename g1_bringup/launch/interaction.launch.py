from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sound_play",
                default_value="true",
                choices=["true", "false"],
                description="Enable sound playback and Festival speech synthesis",
            ),
            DeclareLaunchArgument(
                "sound_device",
                default_value="default",
                description="ALSA output device, e.g. plughw:CARD=Speaker,DEV=0",
            ),
            Node(
                package="sound_play",
                executable="soundplay_node.py",
                name="soundplay_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_sound_play")),
                parameters=[
                    {
                        "device": ParameterValue(
                            LaunchConfiguration("sound_device"), value_type=str
                        ),
                    }
                ],
                respawn=False,
            ),
        ]
    )
