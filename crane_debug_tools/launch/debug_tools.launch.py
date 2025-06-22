# Copyright 2024 Ibis SSL
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

"""Launch file for crane debug tools"""

from launch import LaunchDescription  # type: ignore
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for crane debug tools"""

    # Declare launch arguments
    web_port_arg = DeclareLaunchArgument(
        "web_port", default_value="8080", description="Port for the web bridge server"
    )

    enable_web_arg = DeclareLaunchArgument(
        "enable_web",
        default_value="true",
        description="Enable web-based debugging interface",
    )

    enable_cli_arg = DeclareLaunchArgument(
        "enable_cli",
        default_value="false",
        description="Enable CLI debugging interface",
    )

    # Simple web server
    web_server_node = Node(
        package="crane_debug_tools",
        executable="crane_web_server",
        name="crane_web_server",
        parameters=[{"port": LaunchConfiguration("web_port")}],
        output="screen",
        condition=LaunchConfiguration("enable_web"),
    )

    # CLI skill tester node
    cli_node = Node(
        package="crane_debug_tools",
        executable="crane_skill_cli",
        name="crane_skill_cli",
        output="screen",
        condition=LaunchConfiguration("enable_cli"),
    )

    return LaunchDescription(
        [
            web_port_arg,
            enable_web_arg,
            enable_cli_arg,
            web_server_node,
            cli_node,
        ]
    )
