import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict


def test_avoid_ball(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    for i in range(11):
        rcst_comm.send_yellow_robot(i, -1.0, 3.0 - i * 0.5, math.radians(0))

    def yellow_robot_did_not_avoid_ball(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        for robot in yellow_robots.values():
            if calc.distance_robot_and_ball(robot, ball) < 0.4:
                return True
        return False

    rcst_comm.observer.customized().register_sticky_true_callback(
        "yellow_robot_did_not_avoid_ball", yellow_robot_did_not_avoid_ball
    )

    rcst_comm.change_referee_command("STOP", 1.0)

    def check(x: float, y: float, vx: float = 0.0, vy: float = 0.0):
        rcst_comm.send_ball(x, y, vx, vy)
        time.sleep(2)
        rcst_comm.observer.reset()

        success = True
        for _ in range(5):
            if rcst_comm.observer.customized().get_result("yellow_robot_did_not_avoid_ball"):
                success = False
                break
            time.sleep(1)
        assert success is True

    check(0, 0)
    check(3.5, 0)
    check(3.5, 0, vx=-2.0)
