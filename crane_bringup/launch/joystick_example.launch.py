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

    joy_node = Node(
        package="joy", executable="joy_node", output="screen", parameters=[{"dev": dev}]
    )

    teleop_node = Node(package="crane_teleop", executable="teleop_node", output="screen")

    sim_sender = Node(
        package="crane_sender",
        executable="sim_sender_node",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("crane_sender"), "config", "grsim.yaml")
        ],
    )

    grsim = Node(package="robocup_ssl_comm", executable="grsim_node")

    ibis_sender = Node(
        package="crane_sender",
        executable="ibis_sender_node",
        output="screen",
        parameters=[{"sim": False}],
    )

    ld = LaunchDescription()

    ld.add_action(declare_dev)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    ld.add_action(ibis_sender)
    # ld.add_action(sim_sender)
    ld.add_action(grsim)

    return ld
