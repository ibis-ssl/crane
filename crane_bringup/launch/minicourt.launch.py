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

import os
from struct import pack

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    dev = LaunchConfiguration("dev")
    # sim = LaunchConfiguration('sim')  # TODO : 現在未使用 シミュレータの切り替え用
    # TODO : crane_descriptionからのパラメータ読み込み


declare_dev_cmd = DeclareLaunchArgument(
    "dev", default_value="/dev/input/js0", description="joystick device file"
)

start_joy_node_cmd = Node(
    package="joy", node_executable="joy_node", output="screen", parameters=[{"dev": dev}]
)

start_teleop_node_cmd = Node(package="crane_teleop", node_executable="teleop_node", output="screen")

declare_arg_vision_addr = DeclareLaunchArgument(
    "vision_addr",
    default_value="224.5.23.2",
    description=("Set multicast address to connect SSL-Vision."),
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
)

local_planner = Node(
    package="crane_local_planner", executable="crane_local_planner_node", output="screen"
)

sim_sender = Node(
    package="crane_sender",
    executable="sim_sender_node",
    output="screen",
    parameters=[
        os.path.join(get_package_share_directory("crane_sender"), "config", "grsim.yaml"),
        {"no_movement": True},
    ],
)

ibis_sender = Node(
    package="crane_sender", executable="ibis_sender_node", parameters=[{"sim": False}]
)

waiter = Node(package="crane_planner_plugins", executable="waiter_node")

goalie = Node(package="crane_planner_plugins", executable="goalie_node")

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

grsim = Node(package="robocup_ssl_comm", executable="grsim_node", output="screen")

vision_tracker = Node(package="consai_vision_tracker", executable="vision_tracker_node")

world_model_publisher = Node(
    package="crane_world_model_publisher", executable="crane_world_model_publisher_node"
)

defender = Node(package="crane_planner_plugins", executable="defender_node")

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
        visualizer,
    ]
)
