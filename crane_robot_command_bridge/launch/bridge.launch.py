from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="crane_robot_command_bridge",
                executable="bridge_node",
                name="crane_robot_command_bridge",
                parameters=[{"web_server_port": 8000}, {"web_server_host": "0.0.0.0"}],
                output="screen",
            )
        ]
    )
