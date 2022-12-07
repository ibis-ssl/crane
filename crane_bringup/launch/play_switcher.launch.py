# Copyright (c) 2022 ibis-ssl
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

from struct import pack
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


import os

def generate_launch_description():
    declare_arg_vision_addr = DeclareLaunchArgument(
        'vision_addr', default_value='224.5.23.2',
        description=('Set multicast address to connect SSL-Vision.')
    )

    declare_arg_vision_port = DeclareLaunchArgument(
        'vision_port', default_value='10020',
        description=('Set multicast port to connect SSL-Vision.')
    )

    declare_arg_referee_addr = DeclareLaunchArgument(
        'referee_addr', default_value='224.5.23.1',
        description=('Set multicast address to connect Game Controller.')
    )

    declare_arg_referee_port = DeclareLaunchArgument(
        'referee_port', default_value='11003',
        description=('Set multicast port to connect Game Controller.')
    )


    crane_container = ComposableNodeContainer(
        name='crane_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='crane_world_model_publisher',
                plugin='crane::WorldModelPublisherComponent',
                name='world_model_publisher',
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # ComposableNode(
            #     package='crane_session_controller',
            #     plugin='crane::SessionControllerComponent',
            #     name='session_controller',
            #     # extra_arguments=[{'use_intra_process_comms': True}],
            # ),
            # ComposableNode(
            #     package='crane_local_planner',
            #     plugin='crane::LocalPlannerComponent',
            #     name='local_planner',
            #     # extra_arguments=[{'use_intra_process_comms': True}],
            # ),
            # ComposableNode(
            #     package='crane_sender',
            #     plugin='crane::SimSenderComponent',
            #     name='sim_sender',
            #     # extra_arguments=[{'use_intra_process_comms': True}],
            # ),
            ComposableNode(
                package='crane_play_switcher',
                plugin='crane::PlaySwitcher',
                name='play_switcher',
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    consai_container = ComposableNodeContainer(
        name='consai_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='robocup_ssl_comm',
                plugin='robocup_ssl_comm::Vision',
                name='vision',
                parameters=[{
                    'multicast_address': LaunchConfiguration('vision_addr'),
                    'multicast_port': LaunchConfiguration('vision_port'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='robocup_ssl_comm',
                plugin='robocup_ssl_comm::GameController',
                name='game_controller',
                parameters=[{
                    'multicast_address': LaunchConfiguration('referee_addr'),
                    'multicast_port': LaunchConfiguration('referee_port'),
                }],
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='robocup_ssl_comm',
                plugin='robocup_ssl_comm::GrSim',
                name='grsim'),
            ComposableNode(
                package='consai_vision_tracker',
                plugin='consai_vision_tracker::Tracker',
                name='vision_tracker',
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_arg_vision_addr,
        declare_arg_vision_port,
        declare_arg_referee_addr,
        declare_arg_referee_port,
        consai_container,
        crane_container,
    ])
