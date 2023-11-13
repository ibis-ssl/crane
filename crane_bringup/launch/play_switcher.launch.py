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

import logging
from typing import Text

from launch.frontend import Entity, expose_action, Parser

from launch.actions import EmitEvent
from launch.events import Shutdown as ShutdownEvent
from launch.events.process import ProcessExited
from launch.launch_context import LaunchContext

_logger = logging.getLogger(name='launch')


class ShutdownOnce(EmitEvent):
    shutdown_called = False
    """Action that shuts down a launched system by emitting Shutdown when executed."""

    def __init__(self, *, reason: Text = 'reason not given', **kwargs):
        super().__init__(event=ShutdownEvent(reason=reason), **kwargs)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `Shutdown` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        reason = entity.get_attr('reason', optional=True)
        if reason:
            kwargs['reason'] = parser.parse_substitution(reason)
        return cls, kwargs

    def execute(self, context: LaunchContext):
        """Execute the action."""
        if ShutdownOnce.shutdown_called:
            return
        else:
            ShutdownOnce.shutdown_called = True
            try:
                event = context.locals.event
            except AttributeError:
                event = None

            if isinstance(event, ProcessExited):
                _logger.info('process[{}] was required: shutting down launched system'.format(
                    event.process_name))

            super().execute(context)


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
        on_exit=ShutdownOnce(),
        parameters=[
            {
                "initial_session": "STOP",
                # "event_config_file_name": "normal.yaml"
                "event_config_file_name": "event_config.yaml"
                # "initial_session": "goalie",
            }
        ],
    )

    local_planner = Node(
        package="crane_local_planner",
        executable="crane_local_planner_node",
        output="screen",
        on_exit=ShutdownOnce(),
        parameters=[
            {
                "enable_rvo": False,
                # "non_rvo_gain": 2.15,
                "non_rvo_p_gain": 1.0,
                "non_rvo_d_gain": 1.0,
            }
        ],
    )

    vision = Node(
        package="robocup_ssl_comm",
        executable="vision_node",
        on_exit=ShutdownOnce(),
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
        on_exit=ShutdownOnce(),
        # output="screen",
        parameters=[
            {
                "initial_team_color": "YELLOW",
                "team_name": "ibis",
            }
        ],
    )

    play_switcher = Node(
        package="crane_play_switcher",
        executable="play_switcher_node",
        output="screen"
    )

    visualizer = Node(package="consai_visualizer", executable="consai_visualizer")

    sim_sender = Node(
        package="crane_sender",
        executable="sim_sender_node",
        output="screen",
        parameters=[
            {
                "no_movement": False,
                "theta_kp": 12.0,
                "theta_ki": 0.0,
                "theta_kd": 1.0,
            }
        ],
    )

    real_sender = Node(
        package="crane_sender",
        executable="real_sender_node",
        output="screen",
        parameters=[{}],
    )

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
            # real_sender,
            sim_sender,
            world_model_publisher,
            play_switcher,
            visualizer,
        ]
    )
