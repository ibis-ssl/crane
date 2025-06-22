import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict


def test_robot_robot_collision_avoidance(rcst_comm: Communication):
    """ロボット間衝突回避のテスト"""
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(4.0, 3.0)  # ボールを邪魔にならない場所に配置

    # 衝突コースにロボットを配置
    # Yellow robot が右から左へ移動、Blue robot が上から下へ移動（交差コース）
    rcst_comm.send_yellow_robot(0, 2.0, 0.0, math.radians(180))  # 西向き
    rcst_comm.send_blue_robot(0, 0.0, 2.0, math.radians(270))  # 南向き

    # 追加のロボットで複雑な状況を作成
    rcst_comm.send_yellow_robot(1, -2.0, 0.0, math.radians(0))  # 東向き
    rcst_comm.send_blue_robot(1, 0.0, -2.0, math.radians(90))  # 北向き

    rcst_comm.change_referee_command("FORCE_START", 1.0)

    def detect_collisions(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        """ロボット間の衝突を検出"""
        all_robots = []
        for robot in yellow_robots.values():
            all_robots.append(("yellow", robot))
        for robot in blue_robots.values():
            all_robots.append(("blue", robot))

        collision_detected = False
        min_distance = float("inf")

        for i, (team1, robot1) in enumerate(all_robots):
            for team2, robot2 in all_robots[i + 1 :]:
                distance = calc.distance(robot1.x, robot1.y, robot2.x, robot2.y)
                min_distance = min(min_distance, distance)

                # ロボットの直径を考慮（SSL規格: 最大180mm）
                if distance < 0.18:  # 180mm
                    print(
                        f"Collision detected: {team1} robot vs {team2} robot, distance: {distance:.3f}m"
                    )
                    collision_detected = True

        return collision_detected

    rcst_comm.observer.customized().register_sticky_true_callback(
        "collision_detected", detect_collisions
    )

    # テスト実行
    rcst_comm.observer.reset()
    min_distances = []
    test_duration = 6.0

    for i in range(int(test_duration * 10)):  # 0.1秒間隔で監視
        time.sleep(0.1)

        world = rcst_comm.observer.get_world()
        ball = world.get_ball()
        blue_robots = world.get_blue_robots()
        yellow_robots = world.get_yellow_robots()

        # 最小距離を記録
        all_robots = list(yellow_robots.values()) + list(blue_robots.values())
        for j, robot1 in enumerate(all_robots):
            for robot2 in all_robots[j + 1 :]:
                distance = calc.distance(robot1.x, robot1.y, robot2.x, robot2.y)
                min_distances.append(distance)

    # 結果評価
    collision_occurred = rcst_comm.observer.customized().get_result(
        "collision_detected"
    )
    overall_min_distance = min(min_distances) if min_distances else float("inf")

    print(f"Collision detected during test: {collision_occurred}")
    print(f"Minimum distance between robots: {overall_min_distance:.3f}m")

    # 評価基準
    assert collision_occurred is False, "Robot collision occurred"
    assert overall_min_distance >= 0.18, (
        f"Robots got too close: {overall_min_distance:.3f}m"
    )

    return overall_min_distance


def test_dynamic_obstacle_avoidance(rcst_comm: Communication):
    """動的障害回避のテスト"""
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(0.0, 0.0)

    # テスト対象ロボット（Yellow robot 0）の目標設定
    start_x, start_y = -3.0, 0.0
    goal_x, goal_y = 3.0, 0.0

    rcst_comm.send_yellow_robot(0, start_x, start_y, math.radians(0))

    # 動的障害物（他のロボット）を経路上に配置
    obstacles = [
        (1, -1.0, 0.2, math.radians(270), 0.0, -0.8),  # 下向きに移動
        (2, 0.0, -0.3, math.radians(90), 0.0, 0.6),  # 上向きに移動
        (3, 1.0, 0.1, math.radians(180), -1.5, 0.1),  # 左向きに移動
    ]

    for robot_id, x, y, orientation, target_x, target_y in obstacles:
        rcst_comm.send_yellow_robot(robot_id, x, y, orientation)

    rcst_comm.change_referee_command("FORCE_START", 1.0)

    # 経路追跡と障害回避の監視
    path_points = []
    collision_risks = []
    test_duration = 8.0

    for i in range(int(test_duration * 10)):
        time.sleep(0.1)

        world = rcst_comm.observer.get_world()
        yellow_robots = world.get_yellow_robots()

        if 0 in yellow_robots:
            main_robot = yellow_robots[0]
            path_points.append((main_robot.x, main_robot.y, time.time()))

            # 他のロボットとの距離をチェック
            min_distance_to_obstacle = float("inf")
            for robot_id in [1, 2, 3]:
                if robot_id in yellow_robots:
                    obstacle_robot = yellow_robots[robot_id]
                    distance = calc.distance(
                        main_robot.x, main_robot.y, obstacle_robot.x, obstacle_robot.y
                    )
                    min_distance_to_obstacle = min(min_distance_to_obstacle, distance)

            collision_risks.append(min_distance_to_obstacle)

            # 目標に到達したかチェック
            distance_to_goal = calc.distance(main_robot.x, main_robot.y, goal_x, goal_y)
            if distance_to_goal < 0.3:
                print(f"Goal reached in {i * 0.1:.1f}s")
                break

    # 評価
    if path_points:
        final_pos = path_points[-1]
        distance_to_goal = calc.distance(final_pos[0], final_pos[1], goal_x, goal_y)
        path_length = 0.0

        for i in range(1, len(path_points)):
            segment_length = calc.distance(
                path_points[i - 1][0],
                path_points[i - 1][1],
                path_points[i][0],
                path_points[i][1],
            )
            path_length += segment_length

        min_collision_risk = min(collision_risks) if collision_risks else float("inf")
        avg_collision_risk = (
            sum(collision_risks) / len(collision_risks)
            if collision_risks
            else float("inf")
        )

        print(f"Final distance to goal: {distance_to_goal:.2f}m")
        print(f"Path length: {path_length:.2f}m")
        print(f"Minimum distance to obstacles: {min_collision_risk:.3f}m")
        print(f"Average distance to obstacles: {avg_collision_risk:.3f}m")

        # 評価基準
        assert distance_to_goal <= 0.5, (
            f"Did not reach goal: {distance_to_goal:.2f}m away"
        )
        assert min_collision_risk >= 0.20, (
            f"Got too close to obstacles: {min_collision_risk:.3f}m"
        )
        assert path_length <= 8.0, f"Path too inefficient: {path_length:.2f}m"

        return distance_to_goal, min_collision_risk, path_length
    else:
        assert False, "No path data collected"


def test_wall_boundary_avoidance(rcst_comm: Communication):
    """フィールド境界（壁）回避のテスト"""
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(0.0, 0.0)

    # フィールド境界近くでの動作テスト
    boundary_tests = [
        # (start_x, start_y, target_x, target_y, description)
        (5.8, 0.0, 5.8, 2.0, "Right wall movement"),  # 右壁沿い
        (-5.8, 0.0, -5.8, -2.0, "Left wall movement"),  # 左壁沿い
        (0.0, 2.8, 2.0, 2.8, "Top wall movement"),  # 上壁沿い
        (0.0, -2.8, -2.0, -2.8, "Bottom wall movement"),  # 下壁沿い
    ]

    successful_tests = 0
    total_tests = len(boundary_tests)

    for i, (start_x, start_y, target_x, target_y, description) in enumerate(
        boundary_tests
    ):
        print(f"Test {i + 1}: {description}")

        # ロボットを境界近くに配置
        rcst_comm.send_yellow_robot(0, start_x, start_y, math.radians(0))

        rcst_comm.change_referee_command("FORCE_START", 1.0)
        time.sleep(0.5)

        # 境界違反の監視
        boundary_violations = []
        test_duration = 4.0

        for j in range(int(test_duration * 10)):
            time.sleep(0.1)

            world = rcst_comm.observer.get_world()
            yellow_robots = world.get_yellow_robots()

            if 0 in yellow_robots:
                robot = yellow_robots[0]

                # フィールド境界チェック（SSL規格: 6.0m x 4.0m）
                field_half_width = 6.0
                field_half_height = 4.0

                violations = []
                if abs(robot.x) > field_half_width:
                    violations.append(f"X boundary: {robot.x:.3f}")
                if abs(robot.y) > field_half_height:
                    violations.append(f"Y boundary: {robot.y:.3f}")

                if violations:
                    boundary_violations.extend(violations)

                # 目標到達チェック
                distance_to_target = calc.distance(robot.x, robot.y, target_x, target_y)
                if distance_to_target <= 0.3:
                    break

        # 結果評価
        world = rcst_comm.observer.get_world()
        if 0 in world.get_yellow_robots():
            robot = world.get_yellow_robots()[0]
            final_distance_to_target = calc.distance(
                robot.x, robot.y, target_x, target_y
            )

            print(f"  Final position: ({robot.x:.3f}, {robot.y:.3f})")
            print(f"  Distance to target: {final_distance_to_target:.3f}m")
            print(f"  Boundary violations: {len(boundary_violations)}")

            if len(boundary_violations) == 0 and final_distance_to_target <= 0.5:
                successful_tests += 1
                print("  PASS")
            else:
                print("  FAIL")
                if boundary_violations:
                    print(
                        f"    Violations: {boundary_violations[:3]}..."
                    )  # 最初の3つのみ表示
        else:
            print("  FAIL - Robot not found")

        print()
        time.sleep(1)

    success_rate = successful_tests / total_tests
    print(
        f"Boundary avoidance success rate: {successful_tests}/{total_tests} ({success_rate:.1%})"
    )

    assert success_rate >= 0.75, f"Boundary avoidance insufficient: {success_rate:.1%}"

    return success_rate


if __name__ == "__main__":
    rcst_comm = Communication()

    print("=== Robot-Robot Collision Avoidance Test ===")
    min_distance = test_robot_robot_collision_avoidance(rcst_comm)

    print("\n=== Dynamic Obstacle Avoidance Test ===")
    goal_distance, obstacle_distance, path_length = test_dynamic_obstacle_avoidance(
        rcst_comm
    )

    print("\n=== Wall Boundary Avoidance Test ===")
    boundary_success_rate = test_wall_boundary_avoidance(rcst_comm)

    print("\n=== Collision Avoidance Summary ===")
    print(f"Minimum robot-robot distance: {min_distance:.3f}m")
    print(f"Dynamic obstacle avoidance - Goal distance: {goal_distance:.3f}m")
    print(
        f"Dynamic obstacle avoidance - Min obstacle distance: {obstacle_distance:.3f}m"
    )
    print(f"Boundary avoidance success rate: {boundary_success_rate:.1%}")

    rcst_comm.close()
    print("COLLISION_AVOIDANCE test passed")
