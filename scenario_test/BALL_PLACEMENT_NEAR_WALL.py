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
    # robot 0: ゴールキーパー（自陣ゴール前）
    rcst_comm.send_yellow_robot(0, -6.0, 0.0, 0)
    # robot 1: ball placer（ボール付近に配置）
    rcst_comm.send_yellow_robot(1, placer_x, placer_y, 0)
    # robot 2-4: 追加フィールドプレイヤー
    rcst_comm.send_yellow_robot(2, -2.0, 1.0, 0)
    rcst_comm.send_yellow_robot(3, -2.0, -1.0, 0)
    rcst_comm.send_yellow_robot(4, -3.0, 0.0, 0)


def test_ball_placement_near_wall_x_boundary(rcst_comm: Communication):
    """
    シナリオ1: X軸方向の壁際にボールがある場合のテスト

    初期配置:
    - ボール: (5.8, 0.0) - X軸方向のフィールド境界付近
    - 配置目標: (0.0, 0.0) - フィールド中央

    期待結果:
    - 15秒以内にボールが配置目標から20cm以内に配置される
    """
    rcst_comm.send_empty_world()

    # ボールをX軸方向の壁際に配置（フィールド内だが境界付近）
    ball_x, ball_y = 5.8, 0.0
    rcst_comm.send_ball(ball_x, ball_y)

    setup_robots(rcst_comm, 5.0, 0.0)

    # 配置目標をフィールド中央に設定
    target_x, target_y = 0.0, 0.0
    rcst_comm.set_ball_placement_position(target_x, target_y)

    # レフェリーコマンドを送信してテスト開始
    rcst_comm.change_referee_command("BALL_PLACEMENT_YELLOW", 3.0)

    rcst_comm.observer.reset()
    time.sleep(15)

    # ボールが配置目標から20cm以内にあるか確認
    final_ball_x = rcst_comm.observer.get_world().get_ball().x
    final_ball_y = rcst_comm.observer.get_world().get_ball().y
    dist_to_target = distance(final_ball_x, final_ball_y, target_x, target_y)

    print(f"Initial ball position: ({ball_x}, {ball_y})")
    print(f"Final ball position: ({final_ball_x}, {final_ball_y})")
    print(f"Target position: ({target_x}, {target_y})")
    print(f"Distance to target: {dist_to_target:.3f}m")

    assert dist_to_target < 0.20, (
        f"Ball placement failed: ball is {dist_to_target:.3f}m from target (expected < 0.20m)"
    )


def test_ball_placement_near_wall_y_boundary(rcst_comm: Communication):
    """
    シナリオ2: Y軸方向の壁際にボールがある場合のテスト

    初期配置:
    - ボール: (2.0, 4.3) - Y軸方向のフィールド境界付近
    - 配置目標: (2.0, 2.0)

    期待結果:
    - 15秒以内にボールが配置目標から20cm以内に配置される
    """
    rcst_comm.send_empty_world()

    # ボールをY軸方向の壁際に配置（フィールド内だが境界付近）
    ball_x, ball_y = 2.0, 4.3
    rcst_comm.send_ball(ball_x, ball_y)

    setup_robots(rcst_comm, 2.0, 3.5)

    # 配置目標を設定
    target_x, target_y = 2.0, 2.0
    rcst_comm.set_ball_placement_position(target_x, target_y)

    # レフェリーコマンドを送信してテスト開始
    rcst_comm.change_referee_command("BALL_PLACEMENT_YELLOW", 3.0)

    rcst_comm.observer.reset()
    time.sleep(15)

    # ボールが配置目標から20cm以内にあるか確認
    final_ball_x = rcst_comm.observer.get_world().get_ball().x
    final_ball_y = rcst_comm.observer.get_world().get_ball().y
    dist_to_target = distance(final_ball_x, final_ball_y, target_x, target_y)

    print(f"Initial ball position: ({ball_x}, {ball_y})")
    print(f"Final ball position: ({final_ball_x}, {final_ball_y})")
    print(f"Target position: ({target_x}, {target_y})")
    print(f"Distance to target: {dist_to_target:.3f}m")

    assert dist_to_target < 0.20, (
        f"Ball placement failed: ball is {dist_to_target:.3f}m from target (expected < 0.20m)"
    )


def test_ball_placement_tight_space(rcst_comm: Communication):
    """
    シナリオ3: 回り込みスペースが不足している場合のテスト

    初期配置:
    - ボール: (5.5, 3.0) - 壁と配置目標の間
    - 配置目標: (4.0, 3.0) - ボールから手前方向

    期待結果:
    - ロボットがスタックせずに動作する
    - 15秒以内にボールが配置目標から30cm以内に配置される
      (スペース不足のため精度は若干緩和)
    """
    rcst_comm.send_empty_world()

    # ボールを壁際に配置
    ball_x, ball_y = 5.5, 3.0
    rcst_comm.send_ball(ball_x, ball_y)

    setup_robots(rcst_comm, 4.5, 3.0)

    # 配置目標をボールより手前に設定（フィールド内で到達可能）
    target_x, target_y = 4.0, 3.0
    rcst_comm.set_ball_placement_position(target_x, target_y)

    # レフェリーコマンドを送信してテスト開始
    rcst_comm.change_referee_command("BALL_PLACEMENT_YELLOW", 3.0)

    rcst_comm.observer.reset()

    # ロボットが動いているか確認（スタックしていないか）
    time.sleep(7)
    yellow_robots = rcst_comm.observer.get_world().get_yellow_robots()
    robot = yellow_robots[1]
    initial_robot_x = robot.x
    initial_robot_y = robot.y

    time.sleep(8)  # さらに8秒待機（合計15秒）

    # ロボットが移動したか確認（スタックしていない証拠）
    yellow_robots = rcst_comm.observer.get_world().get_yellow_robots()
    robot = yellow_robots[1]
    final_robot_x = robot.x
    final_robot_y = robot.y
    robot_moved = (
        distance(initial_robot_x, initial_robot_y, final_robot_x, final_robot_y) > 0.1
    )

    # ボールが配置目標に近づいたか確認
    final_ball_x = rcst_comm.observer.get_world().get_ball().x
    final_ball_y = rcst_comm.observer.get_world().get_ball().y
    dist_to_target = distance(final_ball_x, final_ball_y, target_x, target_y)

    print(f"Initial ball position: ({ball_x}, {ball_y})")
    print(f"Final ball position: ({final_ball_x}, {final_ball_y})")
    print(f"Target position: ({target_x}, {target_y})")
    print(f"Distance to target: {dist_to_target:.3f}m")
    print(
        f"Robot moved: {robot_moved} (distance: {distance(initial_robot_x, initial_robot_y, final_robot_x, final_robot_y):.3f}m)"
    )

    # ロボットが動作していることを確認
    assert robot_moved, "Robot appears to be stuck (did not move)"

    # スペース不足のため精度は緩和するが、ある程度は近づいているはず
    assert dist_to_target < 0.30, (
        f"Ball placement failed: ball is {dist_to_target:.3f}m from target (expected < 0.30m)"
    )


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
