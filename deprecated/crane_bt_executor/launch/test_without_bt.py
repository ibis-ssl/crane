# Copyright (c) 2022 ibis-ssl
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
from launch_ros.actions import Node


def generate_launch_description():

    # sim = LaunchConfiguration('sim')  # TODO : 現在未使用 シミュレータの切り替え用
    # TODO : consai2r2_descriptionからのパラメータ読み込み

    receiver_node_cmd = Node(
        package='consai2r2_receiver', node_executable='vision_receiver',
        output='screen',
    )

    start_teleop_node_cmd = Node(
        package='crane_world_observer', node_executable='world_observer',
        output='screen'
    )

    start_sender_cmd = Node(
        package='consai2r2_sender', node_executable='sim_sender',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(
            'consai2r2_sender'), 'config', 'grsim.yaml')]
    )

    ld = LaunchDescription()

    ld.add_action(receiver_node_cmd)
    ld.add_action(start_teleop_node_cmd)
    ld.add_action(start_sender_cmd)

    return ld
