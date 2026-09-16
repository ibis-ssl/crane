"""STOP 中の速度制限シナリオテスト。

座標は Division を決め打ちせず vision の geometry から導出する（field フィクスチャ）。
"""

import math
import time

from field_helpers import Field

# STOP 中の速度上限 [m/s]
STOP_SPEED_LIMIT = 1.5


def test_robot_speed(field: Field):
    rcst_comm = field.comm
    field.send_empty_world()

    ball_x = field.x(0.17)
    field.send_ball(ball_x, 0)
    for robot_id, y in zip(range(11), field.column_y(11, 0.55)):
        field.send_yellow_robot(robot_id, field.x(-0.17), y, math.radians(0))

    rcst_comm.change_referee_command("STOP", 3.0)

    rcst_comm.observer.reset()
    field.send_ball(ball_x, 0, 5.0, 0.0)  # Move the ball
    for _ in range(10):
        if rcst_comm.observer.robot_speed().some_yellow_robots_over(STOP_SPEED_LIMIT):
            velocities = rcst_comm.observer.robot_speed().yellow_max_velocities()
            for robot_id in velocities:
                print(f"Robot {robot_id} has speed {velocities[robot_id]}")
            assert False, f"Yellow robot exceeded {STOP_SPEED_LIMIT} m/s during STOP"
        time.sleep(1)
