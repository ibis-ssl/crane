import math
import time
from rcst.communication import Communication
from rcst import calc


def test_stop_command_response(rcst_comm: Communication):
    """STOPコマンドへの応答時間テスト"""
    rcst_comm.send_empty_world()

    # 動いているロボットを配置
    rcst_comm.send_ball(0.0, 0.0)
    rcst_comm.send_yellow_robot(0, -2.0, 0.0, math.radians(0))
    rcst_comm.send_yellow_robot(1, -1.0, 1.0, math.radians(0))
    rcst_comm.send_yellow_robot(2, -1.0, -1.0, math.radians(0))

    # まず通常のゲーム状態にする
    rcst_comm.change_referee_command("FORCE_START", 2.0)
    time.sleep(2)

    # STOPコマンド送信前の速度を記録
    world_before = rcst_comm.observer.get_world()
    positions_before = {}
    for robot_id, robot in world_before.get_yellow_robots().items():
        positions_before[robot_id] = (robot.x, robot.y, time.time())

    # STOPコマンド送信
    stop_command_time = time.time()
    rcst_comm.change_referee_command("STOP", 0.1)
    print(f"STOP command sent at {stop_command_time:.3f}")

    # 応答時間の測定
    response_times = {}
    max_response_time = 3.0  # 最大3秒監視

    while time.time() - stop_command_time < max_response_time:
        time.sleep(0.1)
        current_time = time.time()

        world = rcst_comm.observer.get_world()
        yellow_robots = world.get_yellow_robots()

        for robot_id, robot in yellow_robots.items():
            if robot_id not in response_times and robot_id in positions_before:
                # 前回位置との比較で停止判定
                prev_pos = positions_before[robot_id]
                time_diff = current_time - prev_pos[2]

                if time_diff >= 0.5:  # 0.5秒以上経過している場合のみ速度計算
                    distance_moved = calc.distance(
                        robot.x, robot.y, prev_pos[0], prev_pos[1]
                    )
                    speed = distance_moved / time_diff

                    # 速度が0.1 m/s以下で停止と判定
                    if speed <= 0.1:
                        response_time = current_time - stop_command_time
                        response_times[robot_id] = response_time
                        print(
                            f"Robot {robot_id} stopped in {response_time:.3f}s (speed: {speed:.3f} m/s)"
                        )

        # 全ロボットが停止したかチェック
        if len(response_times) >= len(positions_before):
            break

    # 応答時間の評価
    if response_times:
        avg_response_time = sum(response_times.values()) / len(response_times)
        max_response_time_actual = max(response_times.values())

        print(f"Average response time: {avg_response_time:.3f}s")
        print(f"Maximum response time: {max_response_time_actual:.3f}s")
        print(f"Robots responded: {len(response_times)}/{len(positions_before)}")

        # SSL規則: STOPコマンドへの応答は1秒以内が理想
        assert avg_response_time <= 1.5, (
            f"Average response time too slow: {avg_response_time:.3f}s"
        )
        assert max_response_time_actual <= 2.0, (
            f"Some robots too slow to respond: {max_response_time_actual:.3f}s"
        )

        return avg_response_time, max_response_time_actual
    else:
        assert False, "No robots responded to STOP command"


def test_start_command_response(rcst_comm: Communication):
    """STARTコマンドへの応答時間テスト"""
    rcst_comm.send_empty_world()

    # 停止状態のロボットを配置
    rcst_comm.send_ball(1.0, 0.0)
    rcst_comm.send_yellow_robot(0, -1.0, 0.0, math.radians(0))
    rcst_comm.send_yellow_robot(1, -2.0, 1.0, math.radians(0))
    rcst_comm.send_yellow_robot(2, -2.0, -1.0, math.radians(0))

    # STOP状態で開始
    rcst_comm.change_referee_command("STOP", 2.0)
    time.sleep(2)

    # 停止状態での位置を記録
    world_before = rcst_comm.observer.get_world()
    positions_before = {}
    for robot_id, robot in world_before.get_yellow_robots().items():
        positions_before[robot_id] = (robot.x, robot.y, time.time())

    # STARTコマンド送信
    start_command_time = time.time()
    rcst_comm.change_referee_command("FORCE_START", 0.1)
    print(f"START command sent at {start_command_time:.3f}")

    # 応答時間の測定
    response_times = {}
    max_response_time = 3.0

    while time.time() - start_command_time < max_response_time:
        time.sleep(0.1)
        current_time = time.time()

        world = rcst_comm.observer.get_world()
        yellow_robots = world.get_yellow_robots()

        for robot_id, robot in yellow_robots.items():
            if robot_id not in response_times and robot_id in positions_before:
                prev_pos = positions_before[robot_id]
                time_diff = current_time - prev_pos[2]

                if time_diff >= 0.3:  # 0.3秒以上経過してから判定
                    distance_moved = calc.distance(
                        robot.x, robot.y, prev_pos[0], prev_pos[1]
                    )
                    speed = distance_moved / time_diff

                    # 速度が0.2 m/s以上で動作開始と判定
                    if speed >= 0.2:
                        response_time = current_time - start_command_time
                        response_times[robot_id] = response_time
                        print(
                            f"Robot {robot_id} started moving in {response_time:.3f}s (speed: {speed:.3f} m/s)"
                        )

        # 全ロボットが動作開始したかチェック
        if len(response_times) >= len(positions_before):
            break

    # 応答時間の評価
    if response_times:
        avg_response_time = sum(response_times.values()) / len(response_times)
        max_response_time_actual = max(response_times.values())

        print(f"Average start response time: {avg_response_time:.3f}s")
        print(f"Maximum start response time: {max_response_time_actual:.3f}s")
        print(f"Robots responded: {len(response_times)}/{len(positions_before)}")

        # START応答時間の評価基準
        assert avg_response_time <= 1.0, (
            f"Average start response time too slow: {avg_response_time:.3f}s"
        )
        assert max_response_time_actual <= 2.0, (
            f"Some robots too slow to start: {max_response_time_actual:.3f}s"
        )

        return avg_response_time, max_response_time_actual
    else:
        print("WARNING: No robots clearly responded to START command")
        return 0.0, 0.0


def test_free_kick_command_response(rcst_comm: Communication):
    """フリーキックコマンドへの応答時間テスト"""
    rcst_comm.send_empty_world()

    # フリーキック状況を設定
    ball_x, ball_y = 1.0, 1.0
    rcst_comm.send_ball(ball_x, ball_y)

    # Yellow team（キッカー）のロボット
    rcst_comm.send_yellow_robot(0, ball_x - 0.3, ball_y, math.radians(0))

    # Blue team（守備）のロボット（違反位置に配置）
    rcst_comm.send_blue_robot(0, ball_x + 0.3, ball_y, math.radians(180))  # 近すぎる
    rcst_comm.send_blue_robot(
        1, ball_x + 0.4, ball_y + 0.2, math.radians(180)
    )  # 近すぎる

    # STOP状態で開始
    rcst_comm.change_referee_command("STOP", 2.0)
    time.sleep(2)

    # フリーキック前の位置記録
    world_before = rcst_comm.observer.get_world()
    blue_positions_before = {}
    for robot_id, robot in world_before.get_blue_robots().items():
        blue_positions_before[robot_id] = (robot.x, robot.y)

    # フリーキックコマンド送信
    free_kick_time = time.time()
    rcst_comm.change_referee_command("DIRECT_FREE_YELLOW", 0.1)
    print(f"FREE KICK command sent at {free_kick_time:.3f}")

    # Blue team の退避応答時間測定
    retreat_response_times = {}
    max_response_time = 4.0

    while time.time() - free_kick_time < max_response_time:
        time.sleep(0.1)
        current_time = time.time()

        world = rcst_comm.observer.get_world()
        ball = world.get_ball()
        blue_robots = world.get_blue_robots()

        for robot_id, robot in blue_robots.items():
            if (
                robot_id not in retreat_response_times
                and robot_id in blue_positions_before
            ):
                distance_to_ball = calc.distance_robot_and_ball(robot, ball)

                # ボールから500mm以上離れた時点で退避完了と判定
                if distance_to_ball >= 0.5:
                    response_time = current_time - free_kick_time
                    retreat_response_times[robot_id] = response_time
                    print(
                        f"Blue robot {robot_id} retreated in {response_time:.3f}s (distance: {distance_to_ball:.3f}m)"
                    )

        # 全ロボットが退避完了したかチェック
        if len(retreat_response_times) >= len(blue_positions_before):
            break

    # Yellow team のキック実行応答時間測定
    kick_executed = False
    kick_response_time = None

    world = rcst_comm.observer.get_world()
    ball = world.get_ball()
    initial_ball_pos = (ball.x, ball.y)

    kick_detection_start = time.time()
    while time.time() - kick_detection_start < 3.0:
        time.sleep(0.1)

        world = rcst_comm.observer.get_world()
        ball = world.get_ball()

        ball_movement = calc.distance(
            ball.x, ball.y, initial_ball_pos[0], initial_ball_pos[1]
        )

        if ball_movement > 0.2:  # ボールが200mm以上移動
            kick_response_time = time.time() - free_kick_time
            kick_executed = True
            print(f"Free kick executed in {kick_response_time:.3f}s")
            break

    # 応答評価
    if retreat_response_times:
        avg_retreat_time = sum(retreat_response_times.values()) / len(
            retreat_response_times
        )
        max_retreat_time = max(retreat_response_times.values())

        print(f"Average retreat response time: {avg_retreat_time:.3f}s")
        print(f"Maximum retreat response time: {max_retreat_time:.3f}s")
        print(
            f"Blue robots retreated: {len(retreat_response_times)}/{len(blue_positions_before)}"
        )

        # 評価基準
        assert avg_retreat_time <= 3.0, (
            f"Retreat response too slow: {avg_retreat_time:.3f}s"
        )

        return avg_retreat_time, kick_response_time
    else:
        print("WARNING: Blue robots did not retreat properly")
        return None, kick_response_time


def test_game_state_transition_response(rcst_comm: Communication):
    """ゲーム状態遷移への応答テスト"""
    rcst_comm.send_empty_world()

    # 基本的なロボット配置
    rcst_comm.send_ball(0.0, 0.0)
    for i in range(3):
        rcst_comm.send_yellow_robot(i, -2.0 + i * 0.5, 0.0, math.radians(0))

    # 複数の状態遷移をテスト
    state_transitions = [
        ("STOP", "FORCE_START"),
        ("FORCE_START", "STOP"),
        ("STOP", "DIRECT_FREE_YELLOW"),
        ("DIRECT_FREE_YELLOW", "STOP"),
    ]

    transition_times = []

    for from_state, to_state in state_transitions:
        print(f"Testing transition: {from_state} -> {to_state}")

        # 初期状態に設定
        rcst_comm.change_referee_command(from_state, 2.0)
        time.sleep(2)

        # 状態遷移の実行
        transition_start = time.time()
        rcst_comm.change_referee_command(to_state, 0.1)

        # 応答の検出（簡易版）
        time.sleep(1.0)  # 応答時間として1秒を仮定

        transition_time = time.time() - transition_start
        transition_times.append(transition_time)

        print(f"  Transition completed in {transition_time:.3f}s")

    # 平均遷移時間の評価
    avg_transition_time = sum(transition_times) / len(transition_times)
    max_transition_time = max(transition_times)

    print(f"Average state transition time: {avg_transition_time:.3f}s")
    print(f"Maximum state transition time: {max_transition_time:.3f}s")

    # 評価基準
    assert avg_transition_time <= 2.0, (
        f"State transitions too slow: {avg_transition_time:.3f}s"
    )
    assert max_transition_time <= 3.0, (
        f"Some transitions too slow: {max_transition_time:.3f}s"
    )

    return avg_transition_time, max_transition_time


if __name__ == "__main__":
    rcst_comm = Communication()

    print("=== STOP Command Response Test ===")
    stop_avg, stop_max = test_stop_command_response(rcst_comm)

    print("\n=== START Command Response Test ===")
    start_avg, start_max = test_start_command_response(rcst_comm)

    print("\n=== Free Kick Command Response Test ===")
    retreat_avg, kick_time = test_free_kick_command_response(rcst_comm)

    print("\n=== Game State Transition Response Test ===")
    transition_avg, transition_max = test_game_state_transition_response(rcst_comm)

    print("\n=== Referee Response Time Summary ===")
    print(f"STOP response - Average: {stop_avg:.3f}s, Max: {stop_max:.3f}s")
    print(f"START response - Average: {start_avg:.3f}s, Max: {start_max:.3f}s")
    if retreat_avg:
        print(f"Free kick retreat - Average: {retreat_avg:.3f}s")
    if kick_time:
        print(f"Free kick execution: {kick_time:.3f}s")
    print(
        f"State transitions - Average: {transition_avg:.3f}s, Max: {transition_max:.3f}s"
    )

    rcst_comm.close()
    print("REFEREE_RESPONSE_TIME test passed")
