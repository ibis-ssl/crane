import time

import pytest
from field_helpers import Field
from rcst.ball import Ball
from rcst.robot import RobotDict

# ボールをゴールラインからどれだけ手前に置くか [m]（ペナルティエリア内）
BALL_INSET_FROM_GOAL_LINE = 0.8


def test_penalty_area_bypass_stability(field: Field):
    rcst_comm = field.comm
    field.send_empty_world()

    ball_x = field.from_goal_line(+1, BALL_INSET_FROM_GOAL_LINE)
    assert field.is_in_penalty_area(ball_x, 0.0), (
        f"ボール ({ball_x:.3f}, 0) がペナルティエリア内にない。"
        f"エリア前縁は x={field.penalty_front_x(+1):.3f}"
    )
    field.send_ball(ball_x, 0.0)

    # 複数機体を中央付近からスタートさせ、敵陣側への移動で
    # ペナルティエリア横断が起きやすい状況を作る
    for robot_id, y in zip(range(8), field.column_y(8, 0.47)):
        field.send_yellow_robot(robot_id, field.x(-0.25), y, 0.0)

    def yellow_enters_penalty(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        del ball, blue_robots
        for robot in yellow_robots.values():
            if field.is_in_penalty_area(robot.x, robot.y):
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
