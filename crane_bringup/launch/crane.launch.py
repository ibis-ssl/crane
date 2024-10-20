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
            # DeclareLaunchArgument('referee_port', default_value='10003'),
            DeclareLaunchArgument("referee_port", default_value="11111"),
            DeclareLaunchArgument("team", default_value="ibis", description="team name"),
            DeclareLaunchArgument(
                "sim", default_value="true", description="Set true if you want to use simulator."
            ),
            DeclareLaunchArgument(
                "original_grsim",
                default_value="false",
                description="Set true if you want to default grsim.",
            ),
            DeclareLaunchArgument("simple_ai", default_value="false", description="a"),
            DeclareLaunchArgument(
                "max_vel", default_value="3.0", description="Set max velocity of robot."
            ),
            DeclareLaunchArgument(
                "gui", default_value="true", description="Set true if you want to use GUI."
            ),
            DeclareLaunchArgument(
                "speak", default_value="true", description="Set true if you want to use speaker."
            ),
            Node(
                condition=UnlessCondition(LaunchConfiguration("simple_ai")),
                package="crane_session_controller",
                executable="crane_session_controller_node",
                output="screen",
                parameters=[
                    {"initial_session": "HALT"},
                    {"event_config_file_name": "normal.yaml"},
                ],
                on_exit=default_exit_behavior,
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("simple_ai")),
                package="crane_simple_ai",
                executable="crane_simple_ai",
                output="screen",
                on_exit=default_exit_behavior,
            ),
            # Group with sim condition
            GroupAction(
                condition=IfCondition(LaunchConfiguration("sim")),
                actions=[
                    Node(
                        package="crane_local_planner",
                        executable="crane_local_planner_node",
                        output="screen",
                        parameters=[
                            {"planner": "rvo2"},
                            {"p_gain": 5.0},
                            {"i_gain": 0.00},
                            {"i_saturation": 0.00},
                            {"d_gain": 1.0},
                            {"max_vel": LaunchConfiguration("max_vel")},
                            {"max_acc": 3.0},
                            {"deceleration_factor": 1.5},
                        ],
                        on_exit=default_exit_behavior,
                    ),
                    Node(
                        package="crane_clock_publisher",
                        executable="crane_clock_publisher_node",
                        output="screen",
                        parameters=[{"time_scale": 1.00}],
                        on_exit=default_exit_behavior,
                    ),
                ],
            ),
            # Group without sim condition
            GroupAction(
                condition=UnlessCondition(LaunchConfiguration("sim")),
                actions=[
                    Node(
                        package="crane_local_planner",
                        executable="crane_local_planner_node",
                        output="screen",
                        parameters=[
                            {"planner": "rvo2"},
                            {"p_gain": 5.5},
                            {"i_gain": 0.0},
                            {"i_saturation": 0.0},
                            {"d_gain": 4.0},
                            {"max_vel": LaunchConfiguration("max_vel")},
                            {"max_acc": 4.0},
                            {"deceleration_factor": 1.5},
                        ],
                        on_exit=default_exit_behavior,
                    )
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("original_grsim")),
                package="crane_sender",
                executable="sim_sender_node",
                output="screen",
                parameters=[
                    {"no_movement": False},
                    {"latency_ms": 0.0},
                    {"k_gain": 1.5},
                    {"i_gain": 0.0},
                    {"d_gain": 1.5},
                    {"theta_k_gain": 2.0},
                    {"theta_i_gain": 0.0},
                    {"theta_d_gain": 0.1},
                    {"kick_power_limit_straight": 0.6},
                    {"kick_power_limit_chip": 1.0},
                    {"sim_mode": "true"},
                ],
                on_exit=default_exit_behavior,
            ),
            Node(
                condition=UnlessCondition(LaunchConfiguration("original_grsim")),
                package="crane_sender",
                executable="ibis_sender_node",
                parameters=[
                    {"no_movement": False},
                    {"latency_ms": 0.0},
                    {"sim_mode": LaunchConfiguration("sim")},
                    {"kick_power_limit_straight": 0.30},
                    {"kick_power_limit_chip": 1.0},
                ],
                on_exit=default_exit_behavior,
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
                package="robocup_ssl_comm", executable="grsim_node", on_exit=default_exit_behavior
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
            # Group with speak condition
            GroupAction(
                condition=IfCondition(LaunchConfiguration("speak")),
                actions=[
                    Node(
                        package="crane_speaker",
                        executable="crane_speaker_node",
                    )
                ],
            ),
            Node(
                package="speak_ros",
                executable="speak_ros_node",
                parameters=[
                    {"plugin_name": "voicevox_plugin::VoiceVoxPlugin"},
                    {"voicevox_plugin/speaker": 13},
                    {"voicevox_plugin/speedScale": 0.8},
                    {"voicevox_plugin/volumeScale": 1.0},
                ],
                on_exit=default_exit_behavior,
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("gui")),
                package="consai_visualizer",
                executable="consai_visualizer",
                on_exit=default_exit_behavior,
            ),
        ]
    )
