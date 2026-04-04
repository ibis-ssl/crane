"""
ボールプレイスメント壁際シナリオテスト

このテストは以下のシナリオを検証します:
1. ボールがフィールド境界付近(壁際)にある場合、正しく壁際処理が行われるか
2. 回り込みスペースが不足している場合、ロボットがスタックせずに動作するか
"""

import math
import time
from rcst.communication import Communication


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """2点間の距離を計算"""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def setup_robots(rcst_comm: Communication, placer_x: float, placer_y: float):
    """ゴールキーパー + ball placerロボットを配置"""
    rcst_comm.send_yellow_robot(0, -6.0, 0.0, 0)   # GK
    rcst_comm.send_yellow_robot(1, placer_x, placer_y, 0)  # ball placer
    rcst_comm.send_yellow_robot(2, -2.0, 1.0, 0)
    rcst_comm.send_yellow_robot(3, -2.0, -1.0, 0)
    rcst_comm.send_yellow_robot(4, -3.0, 0.0, 0)


def run_ball_placement(
    rcst_comm: Communication,
    target_x: float,
    target_y: float,
    timeout: int = 30,
) -> bool:
    """STOP → BALL_PLACEMENT_YELLOW の順でコマンドを送り、成功するまでポーリング"""
    rcst_comm.observer.ball_placement().set_targets(target_x, target_y, for_blue_team=False)

    rcst_comm.change_referee_command("STOP", 3.0)

    rcst_comm.set_ball_placement_position(target_x, target_y)
    rcst_comm.change_referee_command("BALL_PLACEMENT_YELLOW", 0.0)

    rcst_comm.observer.reset()

    for _ in range(timeout):
        if rcst_comm.observer.ball_placement().success():
            return True
        time.sleep(1)
    return False


def test_ball_placement_near_wall_x_boundary(rcst_comm: Communication):
    """
    シナリオ1: X軸方向の壁際にボールがある場合のテスト

    初期配置:
    - ボール: (5.8, 0.0) - X軸方向のフィールド境界付近
    - 配置目標: (0.0, 0.0) - フィールド中央

    期待結果:
    - 30秒以内にボールが配置目標から15cm以内に配置される
    """
    rcst_comm.send_empty_world()
    ball_x, ball_y = 5.8, 0.0
    rcst_comm.send_ball(ball_x, ball_y)
    setup_robots(rcst_comm, 5.0, 0.0)
    time.sleep(1)

    target_x, target_y = 0.0, 0.0
    success = run_ball_placement(rcst_comm, target_x, target_y)

    final_ball = rcst_comm.observer.get_world().get_ball()
    dist = distance(final_ball.x, final_ball.y, target_x, target_y)
    print(f"Initial: ({ball_x}, {ball_y}), Final: ({final_ball.x:.3f}, {final_ball.y:.3f}), "
          f"Target: ({target_x}, {target_y}), Dist: {dist:.3f}m")

    assert success, f"Ball placement failed: ball is {dist:.3f}m from target"


def test_ball_placement_near_wall_y_boundary(rcst_comm: Communication):
    """
    シナリオ2: Y軸方向の壁際にボールがある場合のテスト

    初期配置:
    - ボール: (2.0, 4.3) - Y軸方向のフィールド境界付近
    - 配置目標: (2.0, 2.0)

    期待結果:
    - 30秒以内にボールが配置目標から15cm以内に配置される
    """
    rcst_comm.send_empty_world()
    ball_x, ball_y = 2.0, 4.3
    rcst_comm.send_ball(ball_x, ball_y)
    setup_robots(rcst_comm, 2.0, 3.5)
    time.sleep(1)

    target_x, target_y = 2.0, 2.0
    success = run_ball_placement(rcst_comm, target_x, target_y)

    final_ball = rcst_comm.observer.get_world().get_ball()
    dist = distance(final_ball.x, final_ball.y, target_x, target_y)
    print(f"Initial: ({ball_x}, {ball_y}), Final: ({final_ball.x:.3f}, {final_ball.y:.3f}), "
          f"Target: ({target_x}, {target_y}), Dist: {dist:.3f}m")

    assert success, f"Ball placement failed: ball is {dist:.3f}m from target"


def test_ball_placement_tight_space(rcst_comm: Communication):
    """
    シナリオ3: 回り込みスペースが不足している場合のテスト

    初期配置:
    - ボール: (5.5, 3.0) - 壁際
    - 配置目標: (4.0, 3.0) - ボールより手前

    期待結果:
    - ロボットがスタックせずに動作する
    - 30秒以内にボールが配置目標から15cm以内に配置される
    """
    rcst_comm.send_empty_world()
    ball_x, ball_y = 5.5, 3.0
    rcst_comm.send_ball(ball_x, ball_y)
    setup_robots(rcst_comm, 4.5, 3.0)
    time.sleep(1)

    # robot 1の初期位置を記録してスタック確認用に使う
    initial_robots = rcst_comm.observer.get_world().get_yellow_robots()
    initial_r1_x = initial_robots[1].x
    initial_r1_y = initial_robots[1].y

    target_x, target_y = 4.0, 3.0
    success = run_ball_placement(rcst_comm, target_x, target_y)

    final_ball = rcst_comm.observer.get_world().get_ball()
    dist = distance(final_ball.x, final_ball.y, target_x, target_y)
    final_robots = rcst_comm.observer.get_world().get_yellow_robots()
    robot_moved = distance(initial_r1_x, initial_r1_y, final_robots[1].x, final_robots[1].y) > 0.1

    print(f"Initial: ({ball_x}, {ball_y}), Final: ({final_ball.x:.3f}, {final_ball.y:.3f}), "
          f"Target: ({target_x}, {target_y}), Dist: {dist:.3f}m, Robot moved: {robot_moved}")

    assert robot_moved, "Robot appears to be stuck (did not move)"
    assert success, f"Ball placement failed: ball is {dist:.3f}m from target"


if __name__ == "__main__":
    rcst_comm = Communication()

    print("Running test_ball_placement_near_wall_x_boundary...")
    test_ball_placement_near_wall_x_boundary(rcst_comm)
    print("passed\n")

    print("Running test_ball_placement_near_wall_y_boundary...")
    test_ball_placement_near_wall_y_boundary(rcst_comm)
    print("passed\n")

    print("Running test_ball_placement_tight_space...")
    test_ball_placement_tight_space(rcst_comm)
    print("passed\n")

    rcst_comm.close()
    print("All BALL_PLACEMENT_NEAR_WALL tests passed!")
