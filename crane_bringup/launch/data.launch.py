# Copyright (c) 2024 ibis-ssl
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

default_exit_behavior = Shutdown()


def generate_launch_description():
    return LaunchDescription(
        [
            # Launch Arguments
            DeclareLaunchArgument(
                "vision_addr",
                default_value="224.5.23.2",
                description="Set multicast address to connect SSL-Vision.",
            ),
            DeclareLaunchArgument(
                "vision_port",
                default_value="10006",
                description="Set multicast port to connect SSL-Vision.",
            ),
            DeclareLaunchArgument(
                "referee_addr",
                default_value="224.5.23.1",
                description="Set multicast address to connect Game Controller.",
            ),
            DeclareLaunchArgument("referee_port", default_value="10003"),
            # DeclareLaunchArgument("referee_port", default_value="11111"),
            DeclareLaunchArgument("team", default_value="Test Team", description="team name"),
            DeclareLaunchArgument(
                "gui", default_value="true", description="Set true if you want to use GUI."
            ),
            Node(
                package="robocup_ssl_comm",
                executable="vision_node",
                parameters=[
                    {"multicast_address": LaunchConfiguration("vision_addr")},
                    {"multicast_port": LaunchConfiguration("vision_port")},
                ],
                on_exit=default_exit_behavior,
            ),
            Node(
                package="robocup_ssl_comm",
                executable="game_controller_node",
                parameters=[
                    {"multicast_address": LaunchConfiguration("referee_addr")},
                    {"multicast_port": LaunchConfiguration("referee_port")},
                ],
                on_exit=default_exit_behavior,
            ),
            Node(
                package="crane_robot_receiver",
                executable="robot_receiver_node",
                output="screen",
                # on_exit=default_exit_behavior,
            ),
            Node(
                package="robocup_ssl_comm",
                executable="robot_status_node",
                parameters=[{"blue_port": 10311}, {"yellow_port": 10312}],
                on_exit=default_exit_behavior,
            ),
            Node(
                package="consai_vision_tracker",
                executable="vision_tracker_node",
                on_exit=default_exit_behavior,
            ),
            Node(
                package="crane_world_model_publisher",
                executable="crane_world_model_publisher_node",
                parameters=[
                    {"initial_team_color": "YELLOW"},
                    {"team_name": LaunchConfiguration("team")},
                ],
                on_exit=default_exit_behavior,
            ),
            Node(
                package="crane_play_switcher",
                executable="play_switcher_node",
                output="screen",
                parameters=[
                    {"team_name": LaunchConfiguration("team")},
                ],
                on_exit=Shutdown(),
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("gui")),
                package="consai_visualizer",
                executable="consai_visualizer",
                on_exit=default_exit_behavior,
            ),
        ]
    )
