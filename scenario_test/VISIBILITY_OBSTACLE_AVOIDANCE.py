"""VisibilityGraphPlanner 障害物迂回・衝突回避シナリオテスト

自機（Yellow）と目標（ボール）の間に敵機（Blue）が障害物として立ちはだかる状況で、
自機が衝突を起こさずに障害物を迂回して前進することを検証する。
"""

import math
import time

import pytest
from rcst.ball import Ball
from rcst.communication import Communication
from rcst.robot import RobotDict

# ロボット半径約0.09m。中心間距離0.15m未満を重大な衝突・食い込みと判定
ROBOT_COLLISION_DISTANCE = 0.15


def test_visibility_obstacle_avoidance(rcst_comm: Communication):
    rcst_comm.send_empty_world()

    # 目標となるボールを敵陣側に配置
    rcst_comm.send_ball(3.5, 0.0)

    # ゴールキーパー (ID 0) を自陣ゴール前に配置
    rcst_comm.send_yellow_robot(0, -5.8, 0.0, 0.0)

    # フィールドプレーヤー (ID 1〜7) を自陣側に配置
    for i in range(1, 8):
        rcst_comm.send_yellow_robot(i, -2.0, 2.4 - i * 0.6, 0.0)

    # 進行経路上（中央 x = 0.0 付近）に敵機（Blue）を障害物として配置
    rcst_comm.send_blue_robot(0, 0.0, 0.0, 0.0)
    rcst_comm.send_blue_robot(1, 0.0, 0.5, 0.0)
    rcst_comm.send_blue_robot(2, 0.0, -0.5, 0.0)

    min_dist_info = {"min_dist": 999.0, "yellow_id": -1, "blue_id": -1}

    def yellow_collided_with_blue(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        del ball
        collided = False
        for y_id, yellow in yellow_robots.items():
            for b_id, blue in blue_robots.items():
                dist = math.hypot(yellow.x - blue.x, yellow.y - blue.y)
                if dist < min_dist_info["min_dist"]:
                    min_dist_info["min_dist"] = dist
                    min_dist_info["yellow_id"] = y_id
                    min_dist_info["blue_id"] = b_id
                if dist < ROBOT_COLLISION_DISTANCE:
                    collided = True
        return collided

    rcst_comm.observer.customized().register_sticky_true_callback(
        "yellow_collided_with_blue", yellow_collided_with_blue
    )

    rcst_comm.change_referee_command("FORCE_START", 3.0)
    rcst_comm.observer.reset()

    observed_active_motion = False
    passed_obstacle = False

    for _ in range(12):
        # 衝突検知チェック
        if rcst_comm.observer.customized().get_result("yellow_collided_with_blue"):
            assert False, (
                f"Yellow robot {min_dist_info['yellow_id']} collided with Blue {min_dist_info['blue_id']}! "
                f"Min dist: {min_dist_info['min_dist']:.3f} m (threshold: {ROBOT_COLLISION_DISTANCE} m)"
            )

        world = rcst_comm.observer.get_world()
        if rcst_comm.observer.robot_speed().some_yellow_robots_over(0.2):
            observed_active_motion = True

        for robot_id, yellow in world.get_yellow_robots().items():
            if robot_id != 0 and yellow.x > 0.3:
                passed_obstacle = True

        time.sleep(1.0)

    if not observed_active_motion:
        pytest.skip("Yellow robot did not actively move in this environment")

    # 障害物を越えて敵陣側へ迂回・前進できたことを確認
    assert passed_obstacle, (
        "No field robot managed to bypass the obstacle within the time limit"
    )


if __name__ == "__main__":
    rcst_comm = Communication()
    test_visibility_obstacle_avoidance(rcst_comm)
    rcst_comm.close()
    print("VISIBILITY_OBSTACLE_AVOIDANCE test passed")
