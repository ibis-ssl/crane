"""STOP 中のボール回避シナリオテスト。

座標は Division を決め打ちせず vision の geometry から導出する（field フィクスチャ）。
回避距離 0.4m はルール由来の絶対値なのでそのまま持つ。
"""

import math
import time

from field_helpers import Field
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict

# STOP 中にボールから離れるべき距離 [m]
STOP_KEEP_OUT_DISTANCE = 0.4


def test_avoid_ball(field: Field):
    rcst_comm = field.comm
    field.send_empty_world()
    for robot_id, y in zip(range(11), field.column_y(11, 0.55)):
        field.send_yellow_robot(robot_id, field.x(-0.17), y, math.radians(0))

    def yellow_robot_did_not_avoid_ball(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        for robot in yellow_robots.values():
            if calc.distance_robot_and_ball(robot, ball) < STOP_KEEP_OUT_DISTANCE:
                return True
        return False

    rcst_comm.observer.customized().register_sticky_true_callback(
        "yellow_robot_did_not_avoid_ball", yellow_robot_did_not_avoid_ball
    )

    rcst_comm.change_referee_command("STOP", 1.0)

    def check(x: float, y: float, vx: float = 0.0, vy: float = 0.0):
        field.send_ball(x, y, vx, vy)
        time.sleep(2)
        rcst_comm.observer.reset()

        success = True
        for _ in range(5):
            if rcst_comm.observer.customized().get_result(
                "yellow_robot_did_not_avoid_ball"
            ):
                success = False
                break
            time.sleep(1)
        assert success is True

    check(0, 0)
    check(field.x(0.58), 0)
    check(field.x(0.58), 0, vx=-2.0)
