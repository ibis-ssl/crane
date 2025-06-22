import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict


def test_ball_placement_accuracy(rcst_comm: Communication):
    """ボール配置の精度テスト（指定位置から150mm以内）"""
    rcst_comm.send_empty_world()

    # テスト対象の配置座標
    target_positions = [
        (2.0, 1.5),  # フィールド右上
        (-1.5, -1.0),  # フィールド左下
        (0.0, 0.0),  # 中央
        (3.0, 0.5),  # 右サイド
        (-2.5, 2.0),  # 左上
    ]

    def test_single_placement(target_x: float, target_y: float) -> bool:
        """単一のボール配置テスト"""
        print(f"Testing ball placement at ({target_x}, {target_y})")

        # 初期位置（テスト位置から離れた場所）
        rcst_comm.send_ball(-4.0, -2.0)

        # ボール配置を担当するロボットを配置
        rcst_comm.send_yellow_robot(0, target_x - 0.5, target_y, math.radians(0))

        # BALL_PLACEMENT コマンドを送信（実際の実装に依存）
        rcst_comm.change_referee_command("BALL_PLACEMENT_YELLOW", 1.0)

        # ターゲット位置を設定（レフェリーシステムに依存）
        # 注意: この部分は実際のRCSTの実装に合わせて調整が必要

        # ボール配置の実行を待つ
        time.sleep(5)

        # 最終的なボール位置を取得
        world = rcst_comm.observer.get_world()
        ball = world.get_ball()

        # 精度を計算
        distance_error = calc.distance(ball.x, ball.y, target_x, target_y)
        print(f"  Target: ({target_x}, {target_y})")
        print(f"  Actual: ({ball.x:.3f}, {ball.y:.3f})")
        print(f"  Error: {distance_error:.3f}m")

        # SSL規則: ボール配置の精度は150mm以内
        return distance_error <= 0.15

    # 各位置でのテスト実行
    successful_placements = 0
    total_placements = len(target_positions)

    for target_x, target_y in target_positions:
        if test_single_placement(target_x, target_y):
            successful_placements += 1
            print("  PASS")
        else:
            print("  FAIL")
        print()

    # 結果の評価
    success_rate = successful_placements / total_placements
    print(
        f"Ball placement success rate: {successful_placements}/{total_placements} ({success_rate:.1%})"
    )

    # 最低80%の成功率を要求
    assert success_rate >= 0.8, (
        f"Ball placement accuracy insufficient: {success_rate:.1%}"
    )

    return successful_placements, total_placements


def test_ball_placement_speed(rcst_comm: Communication):
    """ボール配置の速度テスト（制限時間内に完了）"""
    rcst_comm.send_empty_world()

    # 初期位置
    initial_x, initial_y = -3.0, -1.5
    target_x, target_y = 1.0, 2.0

    rcst_comm.send_ball(initial_x, initial_y)
    rcst_comm.send_yellow_robot(0, target_x - 0.3, target_y, math.radians(0))

    # タイマー開始
    start_time = time.time()

    # BALL_PLACEMENT コマンド
    rcst_comm.change_referee_command("BALL_PLACEMENT_YELLOW", 1.0)

    # 配置完了を監視
    placement_completed = False
    max_time = 10.0  # 最大10秒

    while time.time() - start_time < max_time:
        world = rcst_comm.observer.get_world()
        ball = world.get_ball()

        distance_to_target = calc.distance(ball.x, ball.y, target_x, target_y)

        if distance_to_target <= 0.15:  # 150mm以内に到達
            placement_completed = True
            break

        time.sleep(0.1)

    completion_time = time.time() - start_time

    print(f"Ball placement time: {completion_time:.2f}s")
    print(f"Placement completed: {placement_completed}")

    # SSL規則: ボール配置は合理的な時間内（通常10秒以内）に完了する必要がある
    assert placement_completed, "Ball placement did not complete within time limit"
    assert completion_time <= max_time, (
        f"Ball placement took too long: {completion_time:.2f}s"
    )

    return completion_time


def test_ball_placement_obstacle_avoidance(rcst_comm: Communication):
    """障害物回避を含むボール配置テスト"""
    rcst_comm.send_empty_world()

    target_x, target_y = 0.5, 1.0

    # ボールを初期位置に配置
    rcst_comm.send_ball(-2.0, 0.0)

    # 配置担当ロボット
    rcst_comm.send_yellow_robot(0, target_x - 0.4, target_y, math.radians(0))

    # 障害物となるロボットを経路上に配置
    rcst_comm.send_blue_robot(0, -1.0, 0.5, math.radians(0))
    rcst_comm.send_blue_robot(1, -0.5, 0.2, math.radians(0))
    rcst_comm.send_yellow_robot(1, 0.0, 0.8, math.radians(0))

    # BALL_PLACEMENT コマンド
    rcst_comm.change_referee_command("BALL_PLACEMENT_YELLOW", 1.0)

    # 配置の監視（障害物との衝突チェック）
    def check_collisions(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        """ロボット間の衝突チェック"""
        all_robots = list(blue_robots.values()) + list(yellow_robots.values())
        for i, robot1 in enumerate(all_robots):
            for robot2 in all_robots[i + 1 :]:
                distance = calc.distance(robot1.x, robot1.y, robot2.x, robot2.y)
                if distance < 0.18:  # ロボット直径（約180mm）
                    return True
        return False

    rcst_comm.observer.customized().register_sticky_true_callback(
        "collision_detected", check_collisions
    )

    # テスト実行
    rcst_comm.observer.reset()
    time.sleep(8)

    # 結果確認
    world = rcst_comm.observer.get_world()
    ball = world.get_ball()
    distance_error = calc.distance(ball.x, ball.y, target_x, target_y)
    collision_occurred = rcst_comm.observer.customized().get_result(
        "collision_detected"
    )

    print(f"Final ball position: ({ball.x:.3f}, {ball.y:.3f})")
    print(f"Placement accuracy: {distance_error:.3f}m")
    print(f"Collision detected: {collision_occurred}")

    # 評価
    assert distance_error <= 0.15, "Ball placement accuracy insufficient"
    assert collision_occurred is False, "Collision occurred during ball placement"


if __name__ == "__main__":
    rcst_comm = Communication()

    print("=== Ball Placement Accuracy Test ===")
    successful, total = test_ball_placement_accuracy(rcst_comm)

    print("=== Ball Placement Speed Test ===")
    completion_time = test_ball_placement_speed(rcst_comm)

    print("=== Ball Placement Obstacle Avoidance Test ===")
    test_ball_placement_obstacle_avoidance(rcst_comm)

    rcst_comm.close()
    print("BALL_PLACEMENT_ACCURACY test passed")
