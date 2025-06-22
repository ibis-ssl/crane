import math
import time
from rcst.communication import Communication
from rcst import calc


def test_point_to_point_movement(rcst_comm: Communication):
    """ロボットの点間移動精度テスト"""
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(4.0, 3.0)  # ボールを邪魔にならない場所に配置

    # テスト対象の移動経路
    movement_tests = [
        # (start_x, start_y, target_x, target_y, max_error_mm)
        (0.0, 0.0, 2.0, 1.5, 100),  # 中央から右上へ
        (-2.0, -1.0, 1.5, -2.0, 100),  # 左下から右下へ
        (3.0, 0.5, -1.5, 1.8, 150),  # 長距離移動
        (0.5, 2.0, 0.8, 2.3, 50),  # 短距離精密移動
    ]

    successful_movements = 0
    total_movements = len(movement_tests)

    for i, (start_x, start_y, target_x, target_y, max_error_mm) in enumerate(
        movement_tests
    ):
        print(
            f"Movement Test {i + 1}: ({start_x}, {start_y}) -> ({target_x}, {target_y})"
        )

        # ロボットを開始位置に配置
        rcst_comm.send_yellow_robot(0, start_x, start_y, math.radians(0))

        # 移動コマンドを送信（実際の実装では特定のスキルを使用）
        rcst_comm.change_referee_command("FORCE_START", 1.0)

        # 目標位置への移動を待つ
        movement_time = 0
        max_movement_time = 8.0  # 最大8秒
        target_reached = False

        while movement_time < max_movement_time:
            time.sleep(0.2)
            movement_time += 0.2

            world = rcst_comm.observer.get_world()
            if 0 in world.get_yellow_robots():
                robot = world.get_yellow_robots()[0]
                distance_to_target = calc.distance(robot.x, robot.y, target_x, target_y)

                if distance_to_target <= max_error_mm / 1000.0:  # mm to m
                    target_reached = True
                    break

        # 結果評価
        world = rcst_comm.observer.get_world()
        if 0 in world.get_yellow_robots():
            robot = world.get_yellow_robots()[0]
            final_distance = calc.distance(robot.x, robot.y, target_x, target_y)
            final_error_mm = final_distance * 1000

            print(f"  Final position: ({robot.x:.3f}, {robot.y:.3f})")
            print(f"  Error: {final_error_mm:.1f}mm (max: {max_error_mm}mm)")
            print(f"  Time: {movement_time:.1f}s")

            if final_error_mm <= max_error_mm:
                successful_movements += 1
                print("  PASS")
            else:
                print("  FAIL")
        else:
            print("  FAIL - Robot not found")

        print()
        time.sleep(1)  # 次のテストとの間隔

    success_rate = successful_movements / total_movements
    print(
        f"Movement accuracy success rate: {successful_movements}/{total_movements} ({success_rate:.1%})"
    )

    # 最低80%の成功率を要求
    assert success_rate >= 0.8, f"Movement accuracy insufficient: {success_rate:.1%}"

    return successful_movements, total_movements


def test_rotation_accuracy(rcst_comm: Communication):
    """ロボットの回転精度テスト"""
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(3.0, 2.0)

    # 回転テスト（角度の精度）
    rotation_tests = [
        # (initial_angle_deg, target_angle_deg, max_error_deg)
        (0, 90, 5),  # 90度回転
        (45, 180, 5),  # 135度回転
        (270, 0, 8),  # -90度回転（270->0）
        (0, 360, 8),  # 1回転
    ]

    robot_x, robot_y = 0.0, 0.0
    successful_rotations = 0
    total_rotations = len(rotation_tests)

    for i, (initial_deg, target_deg, max_error_deg) in enumerate(rotation_tests):
        print(f"Rotation Test {i + 1}: {initial_deg}° -> {target_deg}°")

        # ロボットを初期角度で配置
        initial_rad = math.radians(initial_deg)
        rcst_comm.send_yellow_robot(0, robot_x, robot_y, initial_rad)

        rcst_comm.change_referee_command("FORCE_START", 1.0)

        # 回転完了を待つ
        rotation_time = 0
        max_rotation_time = 5.0
        target_reached = False

        target_rad = math.radians(target_deg)

        while rotation_time < max_rotation_time:
            time.sleep(0.1)
            rotation_time += 0.1

            world = rcst_comm.observer.get_world()
            if 0 in world.get_yellow_robots():
                robot = world.get_yellow_robots()[0]

                # 角度差を計算（-π to π に正規化）
                angle_diff = target_rad - robot.orientation
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                angle_error_deg = abs(math.degrees(angle_diff))

                if angle_error_deg <= max_error_deg:
                    target_reached = True
                    break

        # 結果評価
        world = rcst_comm.observer.get_world()
        if 0 in world.get_yellow_robots():
            robot = world.get_yellow_robots()[0]
            final_angle_deg = math.degrees(robot.orientation)

            # 最終角度差
            angle_diff = target_deg - final_angle_deg
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360

            final_error_deg = abs(angle_diff)

            print(f"  Final angle: {final_angle_deg:.1f}° (target: {target_deg}°)")
            print(f"  Error: {final_error_deg:.1f}° (max: {max_error_deg}°)")
            print(f"  Time: {rotation_time:.1f}s")

            if final_error_deg <= max_error_deg:
                successful_rotations += 1
                print("  PASS")
            else:
                print("  FAIL")
        else:
            print("  FAIL - Robot not found")

        print()
        time.sleep(1)

    success_rate = successful_rotations / total_rotations
    print(
        f"Rotation accuracy success rate: {successful_rotations}/{total_rotations} ({success_rate:.1%})"
    )

    assert success_rate >= 0.75, f"Rotation accuracy insufficient: {success_rate:.1%}"

    return successful_rotations, total_rotations


def test_velocity_control(rcst_comm: Communication):
    """速度制御の精度テスト"""
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(-3.0, -2.0)

    # 速度制御テスト
    print("Velocity Control Test")

    start_x, start_y = -1.0, 0.0
    rcst_comm.send_yellow_robot(0, start_x, start_y, math.radians(0))

    rcst_comm.change_referee_command("FORCE_START", 1.0)

    # 速度監視
    velocities = []
    test_duration = 3.0
    test_time = 0

    while test_time < test_duration:
        time.sleep(0.1)
        test_time += 0.1

        world = rcst_comm.observer.get_world()
        if 0 in world.get_yellow_robots():
            robot = world.get_yellow_robots()[0]
            # 速度の計算（簡易版 - 実際の実装では robot.velocity を使用）
            velocities.append((robot.x, robot.y, test_time))

    # 速度安定性の評価
    if len(velocities) >= 20:  # 十分なデータポイントがある場合
        # 位置変化から速度を推定
        recent_positions = velocities[-10:]  # 最後の1秒間
        if len(recent_positions) >= 2:
            dx = recent_positions[-1][0] - recent_positions[0][0]
            dy = recent_positions[-1][1] - recent_positions[0][1]
            dt = recent_positions[-1][2] - recent_positions[0][2]

            if dt > 0:
                estimated_speed = math.sqrt(dx * dx + dy * dy) / dt
                print(f"  Estimated speed: {estimated_speed:.2f} m/s")

                # 速度が合理的な範囲内にあることを確認（0.1-3.0 m/s）
                assert 0.1 <= estimated_speed <= 3.0, (
                    f"Speed out of reasonable range: {estimated_speed}"
                )
                print("  Velocity control PASS")
            else:
                print("  Insufficient movement data")
    else:
        print("  Insufficient velocity data collected")


if __name__ == "__main__":
    rcst_comm = Communication()

    print("=== Basic Movement Accuracy Test ===")
    movement_success, movement_total = test_point_to_point_movement(rcst_comm)

    print("=== Rotation Accuracy Test ===")
    rotation_success, rotation_total = test_rotation_accuracy(rcst_comm)

    print("=== Velocity Control Test ===")
    test_velocity_control(rcst_comm)

    # 総合評価
    total_success = movement_success + rotation_success
    total_tests = movement_total + rotation_total
    overall_success_rate = total_success / total_tests

    print("\n=== Overall Results ===")
    print(
        f"Total success rate: {total_success}/{total_tests} ({overall_success_rate:.1%})"
    )

    rcst_comm.close()
    print("BASIC_MOVEMENT_ACCURACY test passed")
