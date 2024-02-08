# Copyright (c) 2019 SSL-Roots
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # parameter
    dev = LaunchConfiguration("dev")
    # sim = LaunchConfiguration('sim')  # TODO : 現在未使用 シミュレータの切り替え用
    # TODO : crane_descriptionからのパラメータ読み込み

    declare_dev = DeclareLaunchArgument(
        "dev", default_value="/dev/input/js0", description="joystick device file"
    )

    # joy_node = Node(
    #     package="joy", executable="joy_node", output="screen", parameters=[{"dev": dev}]
    # )
    #
    # teleop_node = Node(
    #     package="crane_teleop", executable="teleop_node", output="screen"
    # )

    # sim_sender = Node(
    #     package="crane_sender",
    #     executable="sim_sender_node",
    #     output="screen",
    #     parameters=[
    #         os.path.join(get_package_share_directory("crane_sender"), "config", "grsim.yaml")
    #     ],
    # )

    # grsim = Node(package="robocup_ssl_comm", executable="grsim_node")

    ibis_sender = Node(
        package="crane_sender",
        executable="ibis_sender_node",
        output="screen",
        parameters=[{"sim": True}],
    )

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

    vision_tracker = Node(package="consai_vision_tracker", executable="vision_tracker_node")

    world_model_publisher = Node(
        package="crane_world_model_publisher", executable="crane_world_model_publisher_node"
    )

    visualizer = Node(package="consai_visualizer", executable="consai_visualizer", output="screen")

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

    play_switcher = Node(
        package="crane_play_switcher", executable="play_switcher_node", output="screen"
    )

    waiter = Node(package="crane_planner_plugins", executable="waiter_node")

    defender = Node(package="crane_planner_plugins", executable="defender_node")

    goalie = Node(package="crane_planner_plugins", executable="goalie_node")

    session_controller = Node(
        package="crane_session_controller",
        executable="crane_session_controller_node",
        output="screen",
    )

    # local_planner = Node(
    #     package="crane_local_planner",
    #     executable="crane_local_planner_node",
    #     output="screen",
    # )

    ld = LaunchDescription()

    ld.add_action(declare_dev)
    # ld.add_action(joy_node)
    # ld.add_action(teleop_node)
    ld.add_action(real_sender)
    # ld.add_action(grsim)
    ld.add_action(declare_arg_vision_addr)
    ld.add_action(declare_arg_vision_port)
    ld.add_action(vision)
    ld.add_action(vision_tracker)
    ld.add_action(world_model_publisher)
    ld.add_action(visualizer)

    ld.add_action(declare_arg_referee_addr)
    ld.add_action(declare_arg_referee_port)
    ld.add_action(game_controller)

    ld.add_action(play_switcher)
    ld.add_action(session_controller)
    # ld.add_action(local_planner)
    ld.add_action(waiter)
    ld.add_action(defender)
    ld.add_action(goalie)

    return ld
