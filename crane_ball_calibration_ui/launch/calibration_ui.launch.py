"""Ball Calibration UI 起動ファイル."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch description を生成する."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port", default_value="8095", description="Webサーバーのポート番号"
            ),
            DeclareLaunchArgument(
                "host", default_value="0.0.0.0", description="Webサーバーのホスト"
            ),
            Node(
                package="crane_ball_calibration_ui",
                executable="calibration_ui",
                name="ball_calibration_ui",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "host": LaunchConfiguration("host"),
                    }
                ],
            ),
        ]
    )
