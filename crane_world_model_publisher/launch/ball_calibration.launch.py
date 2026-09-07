#!/usr/bin/env python3
# Copyright (c) 2025 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
JSONベースボールモデルキャリブレーション用Launchファイル

このLaunchファイルは、ROSBAGパスをベースにして、そのROSBAGを処理した結果として
生成されるball_calibration_analysisディレクトリ内のJSONデータを使用して
ボール物理モデルのキャリブレーションを実行します。

Usage:
  ros2 launch crane_world_model_publisher ball_calibration.launch.py auto_calibrate:=true
  ros2 launch crane_world_model_publisher ball_calibration.launch.py rosbag_path:=/path/to/rosbag auto_calibrate:=true

自動的に最新のROSBAGを検索し、そのROSBAGをベースにball_calibration_analysisディレクトリを参照します。
"""

import os
from pathlib import Path

import launch
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_latest_rosbag():
    """
    ワークスペース直下の最新rosbagディレクトリを検索

    Returns:
        str: 最新rosbagディレクトリのパス。見つからない場合は空文字列
    """
    # ワークスペースルートディレクトリを取得
    workspace_dir = Path(os.environ.get("ROS_WS_ROOT", os.getcwd()))

    # rosbag2_*パターンのディレクトリを検索
    rosbag_dirs = []
    for path in workspace_dir.glob("rosbag2_*"):
        if path.is_dir():
            metadata_file = path / "metadata.yaml"
            if metadata_file.exists():
                rosbag_dirs.append(path)

    if not rosbag_dirs:
        return ""

    # 最新の修正日時でソート
    latest_rosbag = max(rosbag_dirs, key=lambda x: x.stat().st_mtime)
    return str(latest_rosbag)


def check_json_analysis_exists(rosbag_path):
    """
    ROSBAGパスに対応するball_calibration_analysisディレクトリが存在するかチェック

    Args:
        rosbag_path (str): ROSBAGディレクトリのパス

    Returns:
        bool: ball_calibration_analysisディレクトリとJSONファイルが存在するかどうか
    """
    if not rosbag_path:
        return False

    rosbag_dir = Path(rosbag_path)
    analysis_dir = rosbag_dir / "ball_calibration_analysis"

    if not analysis_dir.is_dir():
        return False

    # JSONファイルが存在するか確認
    json_files = list(analysis_dir.glob("kick_event_visualization_*_data.json"))
    return len(json_files) > 0


def generate_launch_description():
    # 最新ROSBAGの自動検索
    auto_rosbag_path = find_latest_rosbag()

    # Launch引数の宣言
    rosbag_path_arg = DeclareLaunchArgument(
        "rosbag_path",
        default_value=auto_rosbag_path,
        description="キャリブレーション用ROSBAGファイルのパス（空の場合は最新rosbagを自動検索）",
    )

    output_config_path_arg = DeclareLaunchArgument(
        "output_config_path",
        default_value=os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..",
            "calibration",
            "config",
            "calibrated_ball_physics.yaml",
        ),
        description="キャリブレーション結果の出力パス（YAMLファイル）",
    )

    kick_power_analysis_output_arg = DeclareLaunchArgument(
        "kick_power_analysis_output",
        default_value="",  # デフォルトでは自動生成（rosbag_path/ball_calibration_analysis/kick_power_velocity_analysis.json）
        description="キックパワー分析結果のJSON出力パス（空の場合は自動生成）",
    )

    auto_calibrate_arg = DeclareLaunchArgument(
        "auto_calibrate",
        default_value="true",
        description="起動時に自動でキャリブレーションを実行するか",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="ログレベル (DEBUG, INFO, WARN, ERROR)",
    )

    # キャリブレーションノードの起動
    calibration_node = Node(
        package="crane_world_model_publisher",
        executable="ball_calibration_node",
        name="ball_calibration_node",
        output="screen",
        parameters=[
            {
                "rosbag_path": LaunchConfiguration("rosbag_path"),
                "output_config_path": LaunchConfiguration("output_config_path"),
                "kick_power_analysis_output": LaunchConfiguration(
                    "kick_power_analysis_output"
                ),
                "auto_calibrate": LaunchConfiguration("auto_calibrate"),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        remappings=[
            # 必要に応じてトピック名をリマップ
        ],
    )

    # ROSBAGパス確認のログ出力
    if auto_rosbag_path:
        analysis_exists = check_json_analysis_exists(auto_rosbag_path)
        if analysis_exists:
            rosbag_status_msg = f"自動検索で見つかったROSBAGパス: {auto_rosbag_path} (JSON分析データ: 利用可能)"
        else:
            rosbag_status_msg = f"自動検索で見つかったROSBAGパス: {auto_rosbag_path} (警告: ball_calibration_analysisディレクトリが見つかりません)"
    else:
        rosbag_status_msg = "警告: rosbag2_*ディレクトリが見つかりませんでした"

    rosbag_auto_info = LogInfo(msg=rosbag_status_msg)
    rosbag_info = LogInfo(
        msg=["使用するROSBAGパス: ", LaunchConfiguration("rosbag_path")]
    )

    return launch.LaunchDescription(
        [
            rosbag_path_arg,
            output_config_path_arg,
            kick_power_analysis_output_arg,
            auto_calibrate_arg,
            log_level_arg,
            rosbag_auto_info,
            rosbag_info,
            calibration_node,
        ]
    )
