import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict


def test_defensive_formation(rcst_comm: Communication):
    """守備フォーメーションの維持テスト"""
    rcst_comm.send_empty_world()

    # 相手チーム（Blue）の攻撃配置
    rcst_comm.send_ball(2.0, 0.5)
    rcst_comm.send_blue_robot(0, 1.8, 0.3, math.radians(0))  # ボール保持者
    rcst_comm.send_blue_robot(1, 3.0, 1.0, math.radians(270))  # 攻撃サポート
    rcst_comm.send_blue_robot(2, 3.0, -1.0, math.radians(90))  # 攻撃サポート

    # Yellow team の守備フォーメーション（理想的な位置）
    formation_positions = [
        (0, 5.5, 0.0, math.radians(180)),  # ゴールキーパー
        (1, 4.0, 0.8, math.radians(180)),  # センターバック
        (2, 4.0, -0.8, math.radians(180)),  # センターバック
        (3, 3.5, 1.5, math.radians(180)),  # サイドバック
        (4, 3.5, -1.5, math.radians(180)),  # サイドバック
        (5, 2.5, 0.0, math.radians(180)),  # 守備的ミッドフィールダー
    ]

    # ロボットを配置
    for robot_id, x, y, orientation in formation_positions:
        rcst_comm.send_yellow_robot(robot_id, x, y, orientation)

    rcst_comm.change_referee_command("FORCE_START", 2.0)

    def evaluate_formation_quality(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> float:
        """フォーメーションの品質を評価（0.0-1.0）"""
        quality_score = 0.0
        evaluated_robots = 0

        for robot_id, expected_x, expected_y, _ in formation_positions:
            if robot_id in yellow_robots:
                robot = yellow_robots[robot_id]

                # 期待位置からの距離
                distance_error = calc.distance(robot.x, robot.y, expected_x, expected_y)

                # 距離に基づくスコア（500mm以内で満点、1000mm以上で0点）
                if distance_error <= 0.5:
                    robot_score = 1.0
                elif distance_error <= 1.0:
                    robot_score = 1.0 - (distance_error - 0.5) / 0.5
                else:
                    robot_score = 0.0

                quality_score += robot_score
                evaluated_robots += 1

        return quality_score / max(evaluated_robots, 1)

    # フォーメーション維持の監視
    formation_scores = []
    test_duration = 6.0
    measurement_interval = 0.5

    for i in range(int(test_duration / measurement_interval)):
        time.sleep(measurement_interval)

        world = rcst_comm.observer.get_world()
        ball = world.get_ball()
        blue_robots = world.get_blue_robots()
        yellow_robots = world.get_yellow_robots()

        formation_quality = evaluate_formation_quality(ball, blue_robots, yellow_robots)
        formation_scores.append(formation_quality)
        print(
            f"Formation quality at {(i + 1) * measurement_interval:.1f}s: {formation_quality:.2f}"
        )

    # 結果評価
    average_formation_quality = sum(formation_scores) / len(formation_scores)
    min_formation_quality = min(formation_scores)

    print(f"Average formation quality: {average_formation_quality:.2f}")
    print(f"Minimum formation quality: {min_formation_quality:.2f}")

    # 評価基準
    assert average_formation_quality >= 0.7, (
        f"Average formation quality insufficient: {average_formation_quality:.2f}"
    )
    assert min_formation_quality >= 0.5, (
        f"Formation broke down too much: {min_formation_quality:.2f}"
    )

    return average_formation_quality, min_formation_quality


def test_offensive_formation(rcst_comm: Communication):
    """攻撃フォーメーションの維持テスト"""
    rcst_comm.send_empty_world()

    # 攻撃シナリオ設定
    rcst_comm.send_ball(-1.0, 0.0)

    # Yellow team の攻撃フォーメーション
    offensive_positions = [
        (0, 5.5, 0.0, math.radians(180)),  # ゴールキーパー（後方に残る）
        (1, -0.8, 0.2, math.radians(0)),  # ボール保持者
        (2, 1.5, 1.0, math.radians(0)),  # 右ウィング
        (3, 1.5, -1.0, math.radians(0)),  # 左ウィング
        (4, 0.5, 0.0, math.radians(0)),  # センターフォワード
        (5, -1.5, 0.5, math.radians(0)),  # 攻撃的ミッドフィールダー
    ]

    # 相手チーム（Blue）の守備配置
    rcst_comm.send_blue_robot(0, -5.5, 0.0, math.radians(0))  # 相手ゴールキーパー
    rcst_comm.send_blue_robot(1, -3.0, 0.8, math.radians(0))  # 守備者
    rcst_comm.send_blue_robot(2, -3.0, -0.8, math.radians(0))  # 守備者

    # ロボットを配置
    for robot_id, x, y, orientation in offensive_positions:
        rcst_comm.send_yellow_robot(robot_id, x, y, orientation)

    rcst_comm.change_referee_command("FORCE_START", 2.0)

    def evaluate_offensive_spread(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> float:
        """攻撃時の展開の良さを評価"""
        if len(yellow_robots) < 4:
            return 0.0

        # 攻撃ゾーンでのロボット分散度を評価
        attacking_robots = []
        for robot_id, robot in yellow_robots.items():
            if robot_id != 0 and robot.x < 3.0:  # GK以外で攻撃ゾーンにいるロボット
                attacking_robots.append(robot)

        if len(attacking_robots) < 3:
            return 0.0

        # ロボット間距離の評価（適度に分散していることが重要）
        total_distance = 0.0
        distance_count = 0

        for i, robot1 in enumerate(attacking_robots):
            for robot2 in attacking_robots[i + 1 :]:
                distance = calc.distance(robot1.x, robot1.y, robot2.x, robot2.y)
                total_distance += distance
                distance_count += 1

        if distance_count == 0:
            return 0.0

        average_distance = total_distance / distance_count

        # 理想的な距離は1.0-2.0m程度
        if 1.0 <= average_distance <= 2.0:
            spread_score = 1.0
        elif 0.5 <= average_distance < 1.0:
            spread_score = (average_distance - 0.5) / 0.5
        elif 2.0 < average_distance <= 3.0:
            spread_score = 1.0 - (average_distance - 2.0) / 1.0
        else:
            spread_score = 0.0

        return spread_score

    # 攻撃フォーメーションの監視
    spread_scores = []
    test_duration = 5.0
    measurement_interval = 0.5

    for i in range(int(test_duration / measurement_interval)):
        time.sleep(measurement_interval)

        world = rcst_comm.observer.get_world()
        ball = world.get_ball()
        blue_robots = world.get_blue_robots()
        yellow_robots = world.get_yellow_robots()

        spread_quality = evaluate_offensive_spread(ball, blue_robots, yellow_robots)
        spread_scores.append(spread_quality)
        print(
            f"Offensive spread at {(i + 1) * measurement_interval:.1f}s: {spread_quality:.2f}"
        )

    # 結果評価
    average_spread = sum(spread_scores) / len(spread_scores) if spread_scores else 0.0

    print(f"Average offensive spread: {average_spread:.2f}")

    # 評価基準
    assert average_spread >= 0.6, (
        f"Offensive formation spread insufficient: {average_spread:.2f}"
    )

    return average_spread


def test_formation_adaptation(rcst_comm: Communication):
    """フォーメーションの適応性テスト（ボール位置変化への対応）"""
    rcst_comm.send_empty_world()

    # 初期配置
    initial_ball_pos = (-2.0, -1.5)
    rcst_comm.send_ball(initial_ball_pos[0], initial_ball_pos[1])

    # Yellow team の基本配置
    rcst_comm.send_yellow_robot(0, 5.5, 0.0, math.radians(180))  # GK
    rcst_comm.send_yellow_robot(1, 3.0, 0.0, math.radians(180))  # CB
    rcst_comm.send_yellow_robot(2, 2.0, 1.0, math.radians(180))  # 右サイド
    rcst_comm.send_yellow_robot(3, 2.0, -1.0, math.radians(180))  # 左サイド
    rcst_comm.send_yellow_robot(4, 1.0, 0.0, math.radians(180))  # MF

    rcst_comm.change_referee_command("FORCE_START", 2.0)
    time.sleep(2)

    # 初期ポジションを記録
    world = rcst_comm.observer.get_world()
    initial_positions = {}
    for robot_id, robot in world.get_yellow_robots().items():
        initial_positions[robot_id] = (robot.x, robot.y)

    print("Initial positions recorded")

    # ボール位置を大きく変更
    new_ball_pos = (2.0, 2.0)
    rcst_comm.send_ball(new_ball_pos[0], new_ball_pos[1])
    print(f"Ball moved from {initial_ball_pos} to {new_ball_pos}")

    # 適応を待つ
    time.sleep(4)

    # 最終ポジションをチェック
    world = rcst_comm.observer.get_world()
    final_positions = {}
    adaptations = {}

    for robot_id, robot in world.get_yellow_robots().items():
        final_positions[robot_id] = (robot.x, robot.y)

        if robot_id in initial_positions:
            initial_pos = initial_positions[robot_id]
            movement_distance = calc.distance(
                initial_pos[0], initial_pos[1], robot.x, robot.y
            )
            adaptations[robot_id] = movement_distance

    # 適応度の評価
    total_movement = sum(adaptations.values())
    avg_movement = total_movement / len(adaptations) if adaptations else 0.0

    print("Robot position adaptations:")
    for robot_id in sorted(adaptations.keys()):
        print(f"  Robot {robot_id}: {adaptations[robot_id]:.2f}m")

    print(f"Average movement per robot: {avg_movement:.2f}m")
    print(f"Total team movement: {total_movement:.2f}m")

    # 評価基準
    # 少なくとも何らかの適応（移動）が行われることを期待
    assert avg_movement >= 0.3, (
        f"Insufficient formation adaptation: {avg_movement:.2f}m average movement"
    )
    assert total_movement >= 1.0, (
        f"Team did not adapt sufficiently: {total_movement:.2f}m total movement"
    )

    # 過度な移動も問題（フォーメーションの崩壊）
    assert avg_movement <= 2.0, (
        f"Excessive movement suggests formation breakdown: {avg_movement:.2f}m"
    )

    return avg_movement, total_movement


if __name__ == "__main__":
    rcst_comm = Communication()

    print("=== Defensive Formation Test ===")
    def_avg, def_min = test_defensive_formation(rcst_comm)

    print("\n=== Offensive Formation Test ===")
    off_spread = test_offensive_formation(rcst_comm)

    print("\n=== Formation Adaptation Test ===")
    adapt_avg, adapt_total = test_formation_adaptation(rcst_comm)

    print("\n=== Formation Maintenance Summary ===")
    print(f"Defensive formation average: {def_avg:.2f}")
    print(f"Defensive formation minimum: {def_min:.2f}")
    print(f"Offensive spread: {off_spread:.2f}")
    print(f"Adaptation capability: {adapt_avg:.2f}m average movement")

    rcst_comm.close()
    print("FORMATION_MAINTENANCE test passed")
