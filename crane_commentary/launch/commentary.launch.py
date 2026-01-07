# Copyright (c) 2024 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Launch file for SSL Commentary System."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    pkg_dir = get_package_share_directory("crane_commentary")
    config_file = os.path.join(pkg_dir, "config", "commentary_config.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gemini_api_key",
                default_value="",
                description="Gemini API key (can also be set via GEMINI_API_KEY env var)",
            ),
            Node(
                package="crane_commentary",
                executable="commentary_node.py",
                name="commentary_node",
                parameters=[
                    config_file,
                    {
                        "gemini_api_key": LaunchConfiguration("gemini_api_key"),
                    },
                ],
                output="screen",
            ),
        ]
    )
