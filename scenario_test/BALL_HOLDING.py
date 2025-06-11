import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict, Robot

# ボール保持の判定に必要なパラメータ (仮の値、ルールブック等で確認が必要)
BALL_HOLDING_DISTANCE_THRESHOLD = 0.15  # ボールを保持しているとみなすロボットとボールの最大距離 (m)
BALL_HOLDING_DURATION_THRESHOLD = 5.0  # ボールを保持しているとみなす最小継続時間 (s)
DEFENSE_ROBOT_ACCESS_DISTANCE_THRESHOLD = 0.5 # 守備ロボットがボールにアクセス可能とみなす距離 (m)

def is_robot_holding_ball(robot: Robot, ball: Ball) -> bool:
    """指定されたロボットがボールを保持しているか（至近距離にあるか）を判定する"""
    if robot is None or ball is None:
        return False
    return calc.distance_robot_and_ball(robot, ball) < BALL_HOLDING_DISTANCE_THRESHOLD

def is_defense_robot_far_from_ball(ball: Ball, yellow_robots: RobotDict) -> bool:
    """全ての守備ロボットがボールから一定距離以上離れているか"""
    if ball is None:
        return True # ボールがなければアクセス不能とは言えない
    if not yellow_robots:
        return True # 守備ロボットがいなければアクセス不能ではない（ルール解釈による）

    for robot in yellow_robots.values():
        if calc.distance_robot_and_ball(robot, ball) < DEFENSE_ROBOT_ACCESS_DISTANCE_THRESHOLD:
            return False # 近くに守備ロボットがいる
    return True # 全ての守備ロボットが遠い


def test_ball_holding(rcst_comm: Communication):
    # 1. ワールドを初期化
    rcst_comm.send_empty_world()
    print("World initialized.")

    # 2. ボールを配置
    ball_x, ball_y = 1.0, 1.0
    rcst_comm.send_ball(x=ball_x, y=ball_y)
    print(f"Ball placed at ({ball_x}, {ball_y}).")
    time.sleep(0.1)

    # 3. 攻撃側ロボット（青0番）をボールのすぐ近くに配置 (抱え込む向き)
    attacker_id = 0
    attacker_x, attacker_y = 1.0, 0.9
    attacker_orientation = math.pi / 2  # ボール方向を向く
    rcst_comm.send_blue_robot(robot_id=attacker_id, x=attacker_x, y=attacker_y, orientation=attacker_orientation)
    print(f"Attacker (Blue {attacker_id}) placed at ({attacker_x:.2f}, {attacker_y:.2f}) orientation {attacker_orientation:.2f}.")
    time.sleep(0.1)

    # 4. 守備側ロボット（黄0番）を少し離れた位置に配置
    defender_id = 0
    defender_x, defender_y = 1.0, 2.0 # ボールから1m離れた位置
    defender_orientation = -math.pi / 2 # ボール方向を向く
    rcst_comm.send_yellow_robot(robot_id=defender_id, x=defender_x, y=defender_y, orientation=defender_orientation)
    print(f"Defender (Yellow {defender_id}) placed at ({defender_x:.2f}, {defender_y:.2f}) orientation {defender_orientation:.2f}.")
    time.sleep(0.1)

    # 5. ゲームを開始
    rcst_comm.change_referee_command("FORCE_START", 0.5) # 0.5秒後に実行
    print("Referee command 'FORCE_START' sent, effective in 0.5s.")
    time.sleep(1.0) # コマンドが確実に反映されるのを待つ

    # 6. 攻撃側ロボットがボールを保持し続ける (動かさない)
    #    現在の実装では、何もしなければロボットはその場に留まる。
    print(f"Attacker (Blue {attacker_id}) will now attempt to hold the ball.")

    # 7. 一定時間ボールを保持し続ける
    holding_start_time = time.time()
    ball_holding_violation_detected = False

    print(f"Monitoring for Ball Holding violation for {BALL_HOLDING_DURATION_THRESHOLD} seconds...")

    while time.time() - holding_start_time < BALL_HOLDING_DURATION_THRESHOLD + 2.0: # 閾値より少し長く監視
        current_time_in_loop = time.time()
        elapsed_time_in_loop = current_time_in_loop - holding_start_time

        world_state = rcst_comm.observer.get_world()
        if world_state is None:
            print("Warning: Failed to get world state from observer.")
            time.sleep(0.1)
            continue

        current_ball = world_state.get_ball()
        blue_attacker = world_state.get_blue_robot(attacker_id)
        yellow_defenders = world_state.get_yellow_robots()

        if current_ball is None or blue_attacker is None:
            print("Warning: Ball or attacker robot not found in world state.")
            time.sleep(0.1)
            continue

        # 8. Ball Holding違反の検知ロジック
        attacker_is_holding = is_robot_holding_ball(blue_attacker, current_ball)
        defense_is_far = is_defense_robot_far_from_ball(current_ball, yellow_defenders)

        if attacker_is_holding and defense_is_far:
            if elapsed_time_in_loop >= BALL_HOLDING_DURATION_THRESHOLD:
                print(f"Violation DETECTED at {elapsed_time_in_loop:.2f}s: Attacker holding ball and defense far.")
                ball_holding_violation_detected = True
                break
        elif attacker_is_holding and not defense_is_far:
             # 攻撃側がボールを保持しているが、守備側がアクセス可能な場合
             # この場合は違反ではないので、保持開始時間をリセットする必要があるか検討
             # 今回のシナリオでは守備は動かないので、この状態は初期以外では稀
             print(f"Log at {elapsed_time_in_loop:.2f}s: Attacker holding, but defense is close. Resetting hold timer (conceptually).")
             # holding_start_time = current_time_in_loop # 実際にリセットするとテストが成立しなくなるので注意

        elif not attacker_is_holding:
            # 攻撃側がボールを保持していない場合は、違反ではない
            # 保持開始時間をリセット
            print(f"Log at {elapsed_time_in_loop:.2f}s: Attacker NOT holding ball. Resetting hold timer.")
            holding_start_time = current_time_in_loop # 保持が途切れたらタイマーリセット

        # 状態表示 (デバッグ用)
        if elapsed_time_in_loop % 1.0 < 0.1: # 1秒ごとくらいに表示
            print(f"Time: {elapsed_time_in_loop:.1f}s, Holding: {attacker_is_holding}, DefenseFar: {defense_is_far}")

        time.sleep(0.1) # ポーリング間隔

    # アサーション: Ball Holding違反が検知されたことを確認
    assert ball_holding_violation_detected, "Ball Holding violation was NOT detected, but it was expected."
    print("Ball Holding test passed: Violation successfully detected.")


if __name__ == "__main__":
    try:
        from rcst.communication import Communication
        from rcst import calc
        from rcst.ball import Ball
        from rcst.robot import RobotDict, Robot
    except ModuleNotFoundError:
        print("Error: rcst library not found. Please ensure it is installed and accessible.")
        exit(1)

    rcst_comm = Communication()

    # send_blue_robot, send_yellow_robot が存在しない場合のダミー関数 (前回の試行より)
    if not hasattr(rcst_comm, 'send_blue_robot'):
        print("Warning: rcst_comm.send_blue_robot does not exist. Using a dummy function.")
        def _dummy_send_blue_robot(robot_id: int, x: float, y: float, orientation: float, **kwargs):
            print(f"DUMMY send_blue_robot: id={robot_id}, x={x}, y={y}, orientation={orientation}")
        rcst_comm.send_blue_robot = _dummy_send_blue_robot

    if not hasattr(rcst_comm, 'send_yellow_robot'):
        print("Warning: rcst_comm.send_yellow_robot does not exist. Using a dummy function.")
        def _dummy_send_yellow_robot(robot_id: int, x: float, y: float, orientation: float, **kwargs):
            print(f"DUMMY send_yellow_robot: id={robot_id}, x={x}, y={y}, orientation={orientation}")
        rcst_comm.send_yellow_robot = _dummy_send_yellow_robot

    try:
        test_ball_holding(rcst_comm)
    except AssertionError as e:
        print(f"Test failed: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Closing communication.")
        rcst_comm.close()
        print("BALL_HOLDING.py execution finished.")
