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
from ament_index_python.packages import get_package_share_directory
import os

default_exit_behavior = Shutdown()


def generate_launch_description():
    # キッカー物理設定ファイルのパス
    crane_world_model_publisher_share_dir = get_package_share_directory(
        "crane_world_model_publisher"
    )
    kicker_physics_config_path = os.path.join(
        crane_world_model_publisher_share_dir, "config", "kicker_physics.yaml"
    )

    return LaunchDescription(
        [
            # ============================================================
            # ネットワーク設定
            # ============================================================
            DeclareLaunchArgument(
                "vision_addr",
                default_value="224.5.23.2",
                description="SSL-Visionと接続するためのマルチキャストアドレス",
            ),
            DeclareLaunchArgument(
                "vision_port",
                default_value="10006",  # 公式
                # default_value="10020", #独自のやつ
                description="SSL-Visionと接続するためのマルチキャストポート",
            ),
            DeclareLaunchArgument(
                "referee_addr",
                default_value="224.5.23.1",
                description="Game Controllerと接続するためのマルチキャストアドレス",
            ),
            DeclareLaunchArgument(
                "referee_port",
                default_value="10003",  # 公式
                # default_value="11003", # 独自のやつ
                description="Game Controllerと接続するためのマルチキャストポート",
            ),
            DeclareLaunchArgument(
                "tracker_addr",
                default_value="224.5.23.2",
                description="SSL Trackerと接続するためのマルチキャストアドレス",
            ),
            DeclareLaunchArgument(
                "tracker_port",
                default_value="11010",
                description="SSL Trackerと接続するためのマルチキャストポート",
            ),
            # ============================================================
            # チーム・動作モード設定
            # ============================================================
            DeclareLaunchArgument(
                "team",
                default_value="ibis",
                description="チーム名",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="true",
                description="シミュレータフラグ（true: シミュレーション, false: 実機）",
            ),
            # ============================================================
            # デバッグツール設定
            # ============================================================
            DeclareLaunchArgument(
                "debug_tools",
                default_value="true",
                description="デバッグツールの起動フラグ",
            ),
            DeclareLaunchArgument(
                "debug_tools_web",
                default_value="true",
                description="デバッグツールのWebインターフェース有効化",
            ),
            DeclareLaunchArgument(
                "debug_tools_cli",
                default_value="false",
                description="デバッグツールのCLIインターフェース有効化",
            ),
            DeclareLaunchArgument(
                "debug_tools_port",
                default_value="8090",
                description="デバッグツールのWebサーバーポート",
            ),
            # ============================================================
            # ロボット性能パラメータ
            # ============================================================
            DeclareLaunchArgument(
                "max_vel",
                default_value="8.0",
                description="ロボットの最大速度 [m/s]",
            ),
            DeclareLaunchArgument(
                "robot_acc_for_prediction",
                default_value="2.0",
                description="予測計算（slack timeなど）に用いるロボットの加速度 [m/s^2]",
            ),
            DeclareLaunchArgument(
                "robot_max_vel_for_prediction",
                default_value="5.0",
                description="予測計算（slack timeなど）に用いるロボットの最大速度 [m/s]",
            ),
            # ============================================================
            # フィールド・試合設定
            # ============================================================
            DeclareLaunchArgument(
                "is_emplace_positive_side",
                default_value="true",
                description="ロボットの退場方向（true: +X側, false: -X側）",
            ),
            # ============================================================
            # 機能フラグ
            # ============================================================
            DeclareLaunchArgument(
                "speak",
                default_value="false",
                description="音声ノードの起動フラグ",
            ),
            DeclareLaunchArgument(
                "record",
                default_value="true",
                description="rosbag記録フラグ",
            ),
            DeclareLaunchArgument(
                "foxglove",
                default_value="true",
                description="Foxglove Bridge起動フラグ",
            ),
            DeclareLaunchArgument(
                "ball_physics_config_path",
                default_value="grsim_ball_physics.yaml",
                description="ボール物理パラメータ設定ファイル名（configフォルダ内）",
            ),
            # ============================================================
            # コアシステム
            # ============================================================
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
            # ============================================================
            # 可視化・デバッグ系
            # ============================================================
            # WebSocketサーバー
            Node(
                condition=IfCondition(LaunchConfiguration("debug_tools")),
                package="crane_debug_tools",
                executable="crane_websocket_server",
                name="crane_websocket_server",
                parameters=[
                    {
                        "port": LaunchConfiguration("debug_tools_port"),
                        "websocket_port": 8091,
                    }
                ],
                output="screen",
            ),
            # ============================================================
            # プランニング・制御系
            # ============================================================
            # シミュレータ用（local_planner + sender）
            GroupAction(
                condition=IfCondition(LaunchConfiguration("sim")),
                actions=[
                    Node(
                        package="crane_local_planner",
                        executable="crane_local_planner_node",
                        output="screen",
                        parameters=[
                            {"planner": "rvo2"},
                            {"max_vel": LaunchConfiguration("max_vel")},
                            {"max_acc": 2.0},
                            {
                                "acceleration_factor": 1.0
                            },  # 実際の加速度は3.0 * 1.5 = 4.5
                            {"rvo_radius": 0.15},
                            # KickerModel統合：YAML設定ファイルから読み込み
                            {"kicker_physics_config": kicker_physics_config_path},
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
                            {"kick_power_limit_straight": 1.00},
                            {"kick_power_limit_chip": 1.0},
                            {"chip_angle_deg": 30.0},
                            {"theta_p_gain": 3.0},
                            {
                                "use_simple_velocity": False
                            },  # 速度命令でSimpleVelocityを使うかどうか。FalseならPolarVelocityになる
                        ],
                        on_exit=default_exit_behavior,
                    ),
                ],
            ),
            # 実機用（local_planner + sender）
            GroupAction(
                condition=UnlessCondition(LaunchConfiguration("sim")),
                actions=[
                    Node(
                        package="crane_local_planner",
                        executable="crane_local_planner_node",
                        output="screen",
                        parameters=[
                            {"planner": "rvo2"},
                            {"max_vel": LaunchConfiguration("max_vel")},
                            {"max_acc": 2.0},
                            {
                                # "acceleration_factor": 1.3
                                "acceleration_factor": 1.0
                            },  # 実際の加速度は3.0 * 1.5 = 4.5
                            {"straight_kick_power_array": [0.0, 0.4, 1.0, 1.0]},
                            {"straight_kick_speed_array": [0.0, 2.0, 4.0, 7.5]},
                            {"chip_kick_power_array": [0.0, 0.8, 1.0]},
                            {"chip_kick_distance_array": [0.0, 0.5, 1.5]},
                        ],
                        on_exit=default_exit_behavior,
                    ),
                    Node(
                        package="crane_sender",
                        executable="ibis_sender_node",
                        # output="screen",
                        parameters=[
                            {"no_movement": False},
                            {"latency_ms": 100.0},
                            {"sim_mode": LaunchConfiguration("sim")},
                            {"kick_power_limit_straight": 0.50},
                            {"kick_power_limit_chip": 1.0},
                            {
                                "use_simple_velocity": False
                            },  # 速度命令でSimpleVelocityを使うかどうか。FalseならPolarVelocityになる
                            {"acc_limit_offset": 1.0},  # 加速度制限のオフセット値
                        ],
                        on_exit=default_exit_behavior,
                    ),
                ],
            ),
            # ============================================================
            # 通信系
            # ============================================================
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
            # ============================================================
            # モニタリング・診断系
            # ============================================================
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
            # 可視化情報の集約
            Node(
                package="crane_visualization_aggregator",
                executable="crane_visualization_aggregator_node",
                output="screen",
            ),
            # ワールドモデル（世界状態の推定と管理）
            Node(
                package="crane_world_model_publisher",
                executable="crane_world_model_publisher_node",
                parameters=[
                    {"initial_team_color": "YELLOW"},
                    {"team_name": LaunchConfiguration("team")},
                    {"vision_address": LaunchConfiguration("vision_addr")},
                    {"vision_port": LaunchConfiguration("vision_port")},
                    {"tracker_address": LaunchConfiguration("tracker_addr")},
                    {"tracker_port": LaunchConfiguration("tracker_port")},
                    {
                        "is_emplace_positive_side": LaunchConfiguration(
                            "is_emplace_positive_side"
                        )
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
                    {
                        "ball_physics_config_path": LaunchConfiguration(
                            "ball_physics_config_path"
                        ),
                    },
                ],
                output="screen",
                on_exit=default_exit_behavior,
            ),
            # プレイ選択（試合状況に応じた戦略切り替え）
            Node(
                package="crane_play_switcher",
                executable="play_switcher_node",
                output="screen",
                parameters=[
                    {"team_name": LaunchConfiguration("team")},
                ],
                on_exit=Shutdown(),
            ),
            # ============================================================
            # その他機能
            # ============================================================
            # 音声出力（条件付き起動）
            GroupAction(
                condition=IfCondition(LaunchConfiguration("speak")),
                actions=[
                    Node(
                        package="crane_speaker",
                        executable="crane_speaker_node",
                    )
                ],
            ),
            # 音声合成エンジン
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
            # 診断情報の配信
            Node(
                package="crane_robot_receiver",
                executable="diagnostic_publisher_node",
            ),
            # Node(
            #     package="diagnostic_aggregator",
            #     executable="aggregator_node",
            #     output="log",
            # ),
            # ROSバッグ記録（条件付き起動）
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
            # Foxglove Bridge（可視化ツール連携、条件付き起動）
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
                ros_arguments=["--log-level", "warn"],
            ),
        ]
    )
