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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_arg_vision_addr = DeclareLaunchArgument(
        "vision_addr",
        default_value="224.5.23.2",
        description="Set multicast address to connect SSL-Vision.",
    )

    declare_arg_vision_port = DeclareLaunchArgument(
        "vision_port",
        default_value="10006",
        description="Set multicast port to connect SSL-Vision.",
    )

    declare_arg_referee_addr = DeclareLaunchArgument(
        "referee_addr",
        default_value="224.5.23.1",
        description="Set multicast address to connect Game Controller.",
    )

    declare_arg_referee_port = DeclareLaunchArgument(
        "referee_port",
        default_value="11003",
        description="Set multicast port to connect Game Controller.",
    )

    session_controller = Node(
        package="crane_session_controller",
        executable="crane_session_controller_node",
        output="screen",
        parameters=[
            {
                "initial_session": "goalie",
            }
        ],
    )

    local_planner = Node(
        package="crane_local_planner",
        executable="crane_local_planner_node",
        output="screen",
        parameters=[
            {
                "enable_rvo": False,
            }
        ],
    )

    sender = Node(
        package="crane_sender",
        executable="sim_sender_node",
        output="screen",
        parameters=[
            {
                "no_movement": False,
                "theta_kp": 0.7,
                "theta_ki": 0.0,
                "theta_kd": 0.2,
            }
        ],
    )

    waiter = Node(package="crane_planner_plugins", executable="waiter_node")

    goalie = Node(package="crane_planner_plugins", executable="goalie_node", output="screen")

    vision = Node(
        package="robocup_ssl_comm",
        executable="vision_node",
        parameters=[
            {
                "multicast_address": LaunchConfiguration("vision_addr"),
                "multicast_port": LaunchConfiguration("vision_port"),
            }
        ],
    )

    game_controller = Node(
        package="robocup_ssl_comm",
        executable="game_controller_node",
        parameters=[
            {
                "multicast_address": LaunchConfiguration("referee_addr"),
                "multicast_port": LaunchConfiguration("referee_port"),
            }
        ],
    )

    grsim = Node(package="robocup_ssl_comm", executable="grsim_node")

    vision_tracker = Node(package="consai_vision_tracker", executable="vision_tracker_node")

    world_model_publisher = Node(
        package="crane_world_model_publisher",
        executable="crane_world_model_publisher_node",
        output="screen",
        parameters=[
            {
                "initial_team_color": "YELLOW",
            }
        ],
    )

    defender = Node(package="crane_planner_plugins", executable="defender_node", output="screen")

    play_switcher = Node(
        package="crane_play_switcher", executable="play_switcher_node", output="screen"
    )

    visualizer = Node(package="consai_visualizer", executable="consai_visualizer", output="screen")

    return LaunchDescription(
        [
            declare_arg_vision_addr,
            declare_arg_vision_port,
            declare_arg_referee_addr,
            declare_arg_referee_port,
            vision,
            game_controller,
            grsim,
            vision_tracker,
            session_controller,
            local_planner,
            sender,
            defender,
            waiter,
            goalie,
            world_model_publisher,
            play_switcher,
            visualizer,
        ]
    )
