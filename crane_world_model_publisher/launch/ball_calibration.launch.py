#!/usr/bin/env python3

"""
ボールモデルキャリブレーション用Launchファイル
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パッケージディレクトリの取得
    pkg_dir = get_package_share_directory("crane_world_model_publisher")

    # Launch引数の宣言
    rosbag_path_arg = DeclareLaunchArgument(
        "rosbag_path",
        default_value="",
        description="キャリブレーション用ROSBAGファイルのパス",
    )

    output_config_path_arg = DeclareLaunchArgument(
        "output_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("crane_world_model_publisher"),
                "calibration",
                "config",
                "calibrated_ball_physics.yaml",
            ]
        ),
        description="キャリブレーション結果の出力パス",
    )

    auto_calibrate_arg = DeclareLaunchArgument(
        "auto_calibrate",
        default_value="false",
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
                "auto_calibrate": LaunchConfiguration("auto_calibrate"),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        remappings=[
            # 必要に応じてトピック名をリマップ
        ],
    )

    # ROSBAGパス確認のログ出力
    rosbag_info = LogInfo(msg=["ROSBAGパス: ", LaunchConfiguration("rosbag_path")])

    return LaunchDescription(
        [
            rosbag_path_arg,
            output_config_path_arg,
            auto_calibrate_arg,
            log_level_arg,
            rosbag_info,
            calibration_node,
        ]
    )
