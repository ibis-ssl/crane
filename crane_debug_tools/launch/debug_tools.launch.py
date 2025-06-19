"""Launch file for crane debug tools"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for crane debug tools"""
    
    # Declare launch arguments
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='Port for the web bridge server'
    )
    
    enable_web_arg = DeclareLaunchArgument(
        'enable_web',
        default_value='true',
        description='Enable web-based debugging interface'
    )
    
    enable_cli_arg = DeclareLaunchArgument(
        'enable_cli',
        default_value='false',
        description='Enable CLI debugging interface'
    )
    
    # Note: Web bridge server temporarily disabled due to WebSocket++ compatibility issues
    # web_bridge_node = Node(
    #     package='crane_debug_tools',
    #     executable='crane_web_bridge',
    #     name='crane_web_bridge',
    #     parameters=[{
    #         'port': LaunchConfiguration('web_port')
    #     }],
    #     output='screen',
    #     condition=LaunchConfiguration('enable_web')
    # )
    
    # CLI skill tester node
    cli_node = Node(
        package='crane_debug_tools',
        executable='crane_skill_cli',
        name='crane_skill_cli',
        output='screen',
        condition=LaunchConfiguration('enable_cli')
    )
    
    # Static file server for web interface
    web_server_cmd = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8081'],
        cwd=PathJoinSubstitution([
            FindPackageShare('crane_debug_tools'),
            'web'
        ]),
        output='screen',
        condition=LaunchConfiguration('enable_web')
    )
    
    return LaunchDescription([
        web_port_arg,
        enable_web_arg,
        enable_cli_arg,
        # web_bridge_node,  # Temporarily disabled
        cli_node,
        # web_server_cmd,  # Temporarily disabled
    ])