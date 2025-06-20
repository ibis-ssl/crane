"""Launch file for crane debug tools"""

from launch import LaunchDescription
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
