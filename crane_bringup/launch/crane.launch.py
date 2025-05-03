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
from launch.actions import DeclareLaunchArgument, GroupAction, Shutdown, ExecuteProcess
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
                description="SSL-Visionと接続するためのマルチキャストアドレス",
            ),
            DeclareLaunchArgument(
                "vision_port",
                # default_value="10006",
                default_value="10020",
                description="SSL-Visionと接続するためのマルチキャストポート",
            ),
            DeclareLaunchArgument(
                "referee_addr",
                default_value="224.5.23.1",
                description="Game Controllerと接続するためのマルチキャストアドレス",
            ),
            # DeclareLaunchArgument('referee_port', default_value='10003'),
            DeclareLaunchArgument("referee_port", default_value="11003"),
            DeclareLaunchArgument("team", default_value="ibis", description="チーム名"),
            DeclareLaunchArgument(
                "sim", default_value="true", description="シミュレータフラグ"
            ),
            DeclareLaunchArgument(
                "simple_ai", default_value="false", description="SimpleAIモードのフラグ"
            ),
            DeclareLaunchArgument(
                "max_vel", default_value="5.0", description="ロボットの最大速度"
            ),
            DeclareLaunchArgument(
                "speak", default_value="false", description="音声ノードの起動フラグ"
            ),
            DeclareLaunchArgument(
                "is_emplace_positive_side",
                default_value="true",
                description="ロボットの退場する方向",
            ),
            DeclareLaunchArgument(
                "record", default_value="true", description="rosbag記録フラグ"
            ),
            DeclareLaunchArgument(
                "half_court_practice_mode",
                default_value="false",
                description="ハーフコート練習モード",
            ),
            DeclareLaunchArgument(
                "half_court_is_positive_side",
                default_value="true",
                description="ハーフコート練習のサイド",
            ),
            DeclareLaunchArgument(
                "robot_id_mask",
                default_value="",
                description="マスクされたIDは無視され、敵ロボットとみなされる。'1, 2, 3'のようにカンマ区切りで指定する",
            ),
            DeclareLaunchArgument(
                "foxglove",
                default_value="true",
                description="foxglove",
            ),
            DeclareLaunchArgument(
                "robot_acc_for_prediction",
                default_value="2.5",
                description="slack timeの計算などに用いられるロボットの加速度",
            ),
            DeclareLaunchArgument(
                "robot_max_vel_for_prediction",
                default_value="5.0",
                description="slack timeの計算などに用いられるロボットの最大速度",
            ),
            Node(
                package="crane_session_controller",
                executable="crane_session_controller_node",
                output="screen",
                parameters=[
                    {"initial_session": "HALT"},
                    {"event_config_file_name": "normal.yaml"},
                    {
                        "robot_acc_for_prediction": LaunchConfiguration(
                            "robot_acc_for_prediction"
                        ),
                    },
                    {
                        "robot_max_vel_for_prediction": LaunchConfiguration(
                            "robot_max_vel_for_prediction"
                        ),
                    },
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
            # シミュレータ
            GroupAction(
                condition=IfCondition(LaunchConfiguration("sim")),
                actions=[
                    Node(
                        package="crane_local_planner",
                        executable="crane_local_planner_node",
                        output="screen",
                        parameters=[
                            {"planner": "rvo2"},
                            {"p_gain": 3.0},
                            {"i_gain": 0.00},
                            {"i_saturation": 0.00},
                            {"d_gain": 1.0},
                            {"max_vel": LaunchConfiguration("max_vel")},
                            {"max_acc": 2.0},
                            {
                                "acceleration_factor": 1.0
                            },  # 実際の加速度は3.0 * 1.5 = 4.5
                            {"rvo_radius": 0.15},
                            {
                                "half_court_practice_mode": LaunchConfiguration(
                                    "half_court_practice_mode"
                                ),
                            },
                            {
                                "half_court_is_positive_side": LaunchConfiguration(
                                    "half_court_is_positive_side"
                                ),
                            },
                            {"straight_kick_power_array": [0.0, 0.3, 0.6, 1.0]},
                            {"straight_kick_speed_array": [0.0, 1.8, 4.0, 6.0]},
                            {"chip_kick_power_array": [0.0, 0.5, 0.75, 1.0]},
                            {"chip_kick_distance_array": [0.0, 0.3, 1.0, 2.5]},
                        ],
                        on_exit=default_exit_behavior,
                    ),
                    Node(
                        package="crane_sender",
                        # executable="simulation_protocol_sender_node",
                        executable="sim_sender_node",
                        parameters=[
                            {"no_movement": False},
                            {"latency_ms": 0.0},
                            {"sim_mode": LaunchConfiguration("sim")},
                            {"kick_power_limit_straight": 0.50},
                            {"kick_power_limit_chip": 1.0},
                            {"chip_angle_deg": 30.0},
                            {"theta_p_gain": 6.0},
                            {
                                "use_simple_velocity": False
                            },  # 速度命令でSimpleVelocityを使うかどうか。FalseならPolarVelocityになる
                        ],
                        on_exit=default_exit_behavior,
                    ),
                ],
            ),
            # 実機のパラメータ
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
                            {"max_acc": 2.5},
                            {
                                "acceleration_factor": 1.0
                            },  # 実際の加速度は3.0 * 1.5 = 4.5
                            {
                                "half_court_practice_mode": LaunchConfiguration(
                                    "half_court_practice_mode"
                                ),
                            },
                            {
                                "half_court_is_positive_side": LaunchConfiguration(
                                    "half_court_is_positive_side"
                                ),
                            },
                            {"straight_kick_power_array": [0.0, 0.5, 1.0]},
                            {"straight_kick_speed_array": [0.0, 2.5, 7.5]},
                            {"chip_kick_power_array": [0.0, 0.5, 1.0]},
                            {"chip_kick_distance_array": [0.0, 1.0, 2.5]},
                        ],
                        on_exit=default_exit_behavior,
                    ),
                    Node(
                        package="crane_sender",
                        executable="ibis_sender_node",
                        parameters=[
                            {"no_movement": False},
                            {"latency_ms": 100.0},
                            {"sim_mode": LaunchConfiguration("sim")},
                            {"kick_power_limit_straight": 0.50},
                            {"kick_power_limit_chip": 1.0},
                            {
                                "use_simple_velocity": False
                            },  # 速度命令でSimpleVelocityを使うかどうか。FalseならPolarVelocityになる
                        ],
                        on_exit=default_exit_behavior,
                    ),
                ],
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
                package="robocup_ssl_comm",
                executable="grsim_node",
                on_exit=default_exit_behavior,
            ),
            Node(
                package="crane_robot_receiver",
                executable="ping_status_node",
                # output="screen",
                # on_exit=default_exit_behavior,
            ),
            Node(
                package="crane_game_analyzer",
                executable="crane_game_analyzer_node",
                output="screen",
                on_exit=default_exit_behavior,
            ),
            Node(
                # condition=UnlessCondition(LaunchConfiguration("sim")),
                package="crane_robot_receiver",
                executable="robot_receiver_node",
                output="screen",
                respawn=True,
                # on_exit=default_exit_behavior,
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("sim")),
                package="crane_robot_receiver",
                executable="grsim_robot_status_node",
                parameters=[{"blue_port": 30011}, {"yellow_port": 30012}],
            ),
            Node(
                package="crane_visualization_aggregator",
                executable="crane_visualization_aggregator_node",
                output="screen",
            ),
            Node(
                package="crane_world_model_publisher",
                executable="crane_world_model_publisher_node",
                parameters=[
                    {"initial_team_color": "YELLOW"},
                    {"team_name": LaunchConfiguration("team")},
                    {"vision_address": LaunchConfiguration("vision_addr")},
                    {"vision_port": LaunchConfiguration("vision_port")},
                    {"tracker_address": "224.5.23.2"},
                    {"tracker_port": 11010},
                    {
                        "is_emplace_positive_side": LaunchConfiguration(
                            "is_emplace_positive_side"
                        )
                    },
                    {
                        "half_court_practice_mode": LaunchConfiguration(
                            "half_court_practice_mode"
                        ),
                    },
                    {
                        "half_court_is_positive_side": LaunchConfiguration(
                            "half_court_is_positive_side"
                        ),
                    },
                    {
                        "robot_id_mask": LaunchConfiguration("robot_id_mask"),
                    },
                    {
                        "robot_acc_for_prediction": LaunchConfiguration(
                            "robot_acc_for_prediction"
                        ),
                    },
                    {
                        "robot_max_vel_for_prediction": LaunchConfiguration(
                            "robot_max_vel_for_prediction"
                        ),
                    },
                ],
                output="screen",
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
            ),
            Node(
                package="crane_robot_receiver",
                executable="diagnostic_publisher_node",
            ),
            # Node(
            #     package="diagnostic_aggregator",
            #     executable="aggregator_node",
            #     output="log",
            # ),
            # rosbag recordの起動設定
            GroupAction(
                condition=IfCondition(LaunchConfiguration("record")),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "bag",
                            "record",
                            "-a",
                            "-s",
                            "mcap",
                            "--log-level",
                            "fatal",
                        ],
                    ),
                ],
            ),
            # https://github.com/foxglove/ros-foxglove-bridge/blob/main/ros2_foxglove_bridge/launch/foxglove_bridge_launch.xml
            Node(
                condition=IfCondition(LaunchConfiguration("foxglove")),
                package="foxglove_bridge",
                executable="foxglove_bridge",
                parameters=[
                    {"port": 8765},
                    {"address": "0.0.0.0"},
                    {"tls": False},
                    {"certfile": ""},
                    {"keyfile": ""},
                    {"topic_whitelist": [".*"]},
                    {"service_whitelist": [".*"]},
                    {"param_whitelist": [".*"]},
                    {"client_topic_whitelist": [".*"]},
                    {"min_qos_depth": 1},
                    {"max_qos_depth": 10},
                    {"num_threads": 0},
                    {"send_buffer_limit": 10000000},
                    {"use_sim_time": False},
                    {
                        "capabilities": [
                            "clientPublish",
                            "parameters",
                            "parametersSubscribe",
                            "services",
                            "connectionGraph",
                            "assets",
                        ]
                    },
                    {"include_hidden": False},
                    {
                        "asset_uri_allowlist": [
                            "^package://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$"
                        ]
                    },
                ],
                output="log",
                on_exit=default_exit_behavior,
            ),
        ]
    )
