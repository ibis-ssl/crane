import math
import time
from rcst.communication import Communication
from rcst.ball import Ball
from rcst.robot import RobotDict


def is_in_penalty_area(x: float, y: float) -> bool:
    """ペナルティエリア内かどうかを判定 (SSL規格: 1.8m x 1.2m)"""
    return math.fabs(x) >= 4.8 and math.fabs(y) <= 1.2


def test_keeper_area_restriction(rcst_comm: Communication):
    """ゴールキーパー以外のロボットがペナルティエリアに侵入しないことをテスト"""
    rcst_comm.send_empty_world()

    # ボールをペナルティエリア内に配置
    rcst_comm.send_ball(5.5, 0.5)

    # ゴールキーパー（ID=0）をペナルティエリア内に配置
    rcst_comm.send_yellow_robot(0, 5.8, 0.0, math.radians(180))

    # 他のロボットをペナルティエリア近くに配置
    rcst_comm.send_yellow_robot(1, 4.5, 0.5, math.radians(0))
    rcst_comm.send_yellow_robot(2, 4.5, -0.5, math.radians(0))
    rcst_comm.send_yellow_robot(3, 4.0, 1.0, math.radians(0))

    # STOP コマンドを送信
    rcst_comm.change_referee_command("STOP", 2.0)

    def non_keeper_in_penalty_area(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        """ゴールキーパー以外がペナルティエリアに侵入しているかチェック"""
        for robot_id, robot in yellow_robots.items():
            if robot_id != 0 and is_in_penalty_area(robot.x, robot.y):
                print(
                    f"Non-keeper robot {robot_id} in penalty area at ({robot.x:.2f}, {robot.y:.2f})"
                )
                return True
        return False

    rcst_comm.observer.customized().register_sticky_true_callback(
        "non_keeper_in_penalty_area", non_keeper_in_penalty_area
    )

    # テスト実行
    rcst_comm.observer.reset()
    success = True

    for i in range(8):  # 8秒間監視
        if rcst_comm.observer.customized().get_result("non_keeper_in_penalty_area"):
            success = False
            break
        time.sleep(1)

    # ゴールキーパーはペナルティエリア内にいることを確認（正常動作）
    keeper_in_area = False
    world = rcst_comm.observer.get_world()
    if 0 in world.get_yellow_robots():
        keeper = world.get_yellow_robots()[0]
        keeper_in_area = is_in_penalty_area(keeper.x, keeper.y)

    assert success is True, "Non-keeper robots violated penalty area restriction"
    assert keeper_in_area is True, "Goalkeeper should be allowed in penalty area"
    print("Keeper in penalty area (allowed):", keeper_in_area)


if __name__ == "__main__":
    rcst_comm = Communication()
    test_keeper_area_restriction(rcst_comm)
    rcst_comm.close()
    print("STOP_KEEPER_AREA test passed")
