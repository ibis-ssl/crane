import math
import time

import pytest
from rcst.communication import Communication
from rcst.robot import RobotDict
from rcst.ball import Ball


def is_in_penalty_area(x: float, y: float) -> bool:
    # SSL Div-A: field half x=6.0, penalty depth=1.8, half width=1.8
    return math.fabs(x) >= 4.2 and math.fabs(y) <= 1.8


def test_penalty_area_bypass_stability(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(5.2, 0.0)

    # 複数機体を中央付近からスタートさせ、敵陣側への移動で
    # ペナルティエリア横断が起きやすい状況を作る
    for i in range(8):
        rcst_comm.send_yellow_robot(i, -1.5, 2.1 - i * 0.6, 0.0)

    def yellow_enters_penalty(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        del ball, blue_robots
        for robot in yellow_robots.values():
            if is_in_penalty_area(robot.x, robot.y):
                return True
        return False

    rcst_comm.observer.customized().register_sticky_true_callback(
        "yellow_enters_penalty", yellow_enters_penalty
    )

    rcst_comm.change_referee_command("FORCE_START", 3.0)
    rcst_comm.observer.reset()

    # ロボットが実際に動作していることを前提条件として確認
    observed_active_motion = False
    for _ in range(12):
        if rcst_comm.observer.robot_speed().some_yellow_robots_over(0.2):
            observed_active_motion = True
        if rcst_comm.observer.customized().get_result("yellow_enters_penalty"):
            assert False, (
                "Yellow robot entered penalty area while bypass should be active"
            )
        time.sleep(1.0)

    if not observed_active_motion:
        pytest.skip("Robots did not actively move in this environment")


if __name__ == "__main__":
    rcst_comm = Communication()
    test_penalty_area_bypass_stability(rcst_comm)
    rcst_comm.close()
    print("PENALTY_AREA_BYPASS_STABILITY test passed")
