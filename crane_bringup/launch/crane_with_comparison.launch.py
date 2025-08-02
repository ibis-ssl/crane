# Copyright (c) 2025 ibis-ssl
# Launch file for crane with comparison tracker

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            # Launch Arguments
            DeclareLaunchArgument(
                "vision_addr",
                default_value="224.5.23.2",
                description="SSL-Visionと接続するためのマルチキャストアドレス",
            ),
            DeclareLaunchArgument(
                "vision_port",
                default_value="10020",
                description="SSL-Visionと接続するためのマルチキャストポート",
            ),
            DeclareLaunchArgument(
                "referee_addr",
                default_value="224.5.23.1",
                description="Game Controllerと接続するためのマルチキャストアドレス",
            ),
            DeclareLaunchArgument("referee_port", default_value="11003"),
            DeclareLaunchArgument("team", default_value="ibis", description="チーム名"),
            DeclareLaunchArgument(
                "sim", default_value="true", description="シミュレータフラグ"
            ),
            # Include the original crane launch
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("crane_bringup"),
                                "launch",
                                "crane.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "vision_addr": LaunchConfiguration("vision_addr"),
                    "vision_port": LaunchConfiguration("vision_port"),
                    "referee_addr": LaunchConfiguration("referee_addr"),
                    "referee_port": LaunchConfiguration("referee_port"),
                    "team": LaunchConfiguration("team"),
                    "sim": LaunchConfiguration("sim"),
                }.items(),
            ),
            # Add comparison tracker
            Node(
                package="consai_vision_tracker_comparison",
                executable="consai_comparison_tracker",
                name="consai_comparison_tracker",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                    }
                ],
                remappings=[
                    # Input: same vision detection as crane
                    ("/detection", "/detection"),
                    # Output: separate topic for comparison
                    ("/world_model_comparison", "/world_model_comparison"),
                ],
            ),
        ]
    )
