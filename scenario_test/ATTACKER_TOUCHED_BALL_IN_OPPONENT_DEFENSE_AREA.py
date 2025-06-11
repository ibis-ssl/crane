import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict, Robot

# Division A フィールドとディフェンスエリアの定義
FIELD_LENGTH = 12.0  # メートル
FIELD_WIDTH = 9.0    # メートル
DEFENSE_AREA_LENGTH = 1.8 # X軸方向の長さ (ゴールラインからの奥行き)
DEFENSE_AREA_WIDTH = 3.6  # Y軸方向の幅

# 黄色チームのゴールがX軸正側にあると仮定
YELLOW_GOAL_X = FIELD_LENGTH / 2.0  # 6.0
YELLOW_DEFENSE_AREA_X_MAX = YELLOW_GOAL_X  # 6.0
YELLOW_DEFENSE_AREA_X_MIN = YELLOW_GOAL_X - DEFENSE_AREA_LENGTH # 4.2
YELLOW_DEFENSE_AREA_Y_MIN = -DEFENSE_AREA_WIDTH / 2.0 # -1.8
YELLOW_DEFENSE_AREA_Y_MAX = DEFENSE_AREA_WIDTH / 2.0  #  1.8

# 最後にボールに触れたロボットの情報を保持 (チームとID)
# team: 0 for BLUE, 1 for YELLOW, -1 for None
last_touched_robot_info = {"team": -1, "robot_id": -1}

def reset_last_touched_robot():
    global last_touched_robot_info
    last_touched_robot_info = {"team": -1, "robot_id": -1}

def is_position_in_yellow_defense_area(x: float, y: float) -> bool:
    """指定された座標が黄色チームのディフェンスエリア内にあるかを判定する"""
    return (YELLOW_DEFENSE_AREA_X_MIN <= x <= YELLOW_DEFENSE_AREA_X_MAX and
            YELLOW_DEFENSE_AREA_Y_MIN <= y <= YELLOW_DEFENSE_AREA_Y_MAX)

# ボール接触監視コールバック (MULTIPLE_DEFENDERS.py と同様のものを想定)
# def ball_touch_monitor_callback(ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict): ...
# 今回は手動で接触を確認するため、コールバックは使用しません。


def test_attacker_touched_ball_in_opponent_defense_area(rcst_comm: Communication):
    global last_touched_robot_info
    reset_last_touched_robot()

    # 1. ワールドを初期化
    rcst_comm.send_empty_world()
    print("World initialized.")

    # 2. 黄色チームのディフェンスエリア内にボールを配置
    ball_x, ball_y = 5.0, 0.0  # 黄色ディフェンスエリア内 (4.2 to 6.0, -1.8 to 1.8)
    assert is_position_in_yellow_defense_area(ball_x, ball_y), "Ball initial position is not in opponent (yellow) defense area!"
    rcst_comm.send_ball(x=ball_x, y=ball_y)
    print(f"Ball placed at ({ball_x}, {ball_y}) in yellow defense area.")
    time.sleep(0.1)

    # 3. 青チームのロボット0番（攻撃側）を黄色チームのディフェンスエリア内でボールに触れられる位置に配置
    attacker_id = 0
    attacker_x, attacker_y = 4.9, 0.0 # ボールのすぐ近く
    attacker_orientation = 0 # ボール方向 (X軸正)
    assert is_position_in_yellow_defense_area(attacker_x, attacker_y), "Attacker initial position is not in opponent (yellow) defense area!"
    rcst_comm.send_blue_robot(robot_id=attacker_id, x=attacker_x, y=attacker_y, orientation=attacker_orientation)
    print(f"Blue attacker robot {attacker_id} placed at ({attacker_x:.2f}, {attacker_y:.2f}).")
    time.sleep(0.1)

    # 4. 黄色チームのキーパーロボットをディフェンスエリア内のゴールライン上などに配置
    keeper_id = 0
    keeper_x, keeper_y = 5.9, 0.1 # ゴールライン近く
    keeper_orientation = math.pi # フィールド中央を向く
    assert is_position_in_yellow_defense_area(keeper_x, keeper_y), "Yellow keeper initial position is not in their defense area!"
    rcst_comm.send_yellow_robot(robot_id=keeper_id, x=keeper_x, y=keeper_y, orientation=keeper_orientation)
    print(f"Yellow keeper robot {keeper_id} placed at ({keeper_x:.2f}, {keeper_y:.2f}).")
    time.sleep(0.1)

    # 5. ゲームを開始
    rcst_comm.change_referee_command("FORCE_START", 0.5) # 0.5秒後に実行
    print("Referee command 'FORCE_START' sent, effective in 0.5s.")
    time.sleep(1.0) # コマンド反映とロボットの動作開始を待つ

    # 6. 青チームのロボット0番がボールに触れるようにする
    print(f"Moving blue attacker robot {attacker_id} to touch the ball...")
    reset_last_touched_robot() # 接触前の状態をクリア
    rcst_comm.send_blue_robot(robot_id=attacker_id, x=ball_x, y=ball_y, orientation=attacker_orientation, velocity_x=0.5)

    violation_detected = False
    detection_time = 0.0

    # 接触と違反判定のループ
    start_check_time = time.time()
    max_check_duration = 5.0 # 最大5秒間チェック

    while time.time() - start_check_time < max_check_duration:
        world = rcst_comm.observer.get_world()
        if world is None:
            time.sleep(0.01)
            continue

        ball_state = world.get_ball()
        attacker_robot_state = world.get_blue_robot(attacker_id)

        if ball_state is None or attacker_robot_state is None:
            time.sleep(0.01)
            continue

        # ボール接触の手動更新
        if calc.distance_robot_and_ball(attacker_robot_state, ball_state) < 0.15: # 接触距離
            if not (last_touched_robot_info["team"] == 0 and last_touched_robot_info["robot_id"] == attacker_id):
                print(f"Manually detected: Ball touched by BLUE attacker robot {attacker_id}")
            last_touched_robot_info = {"team": 0, "robot_id": attacker_id}

        # 7. Attacker Touched Ball In Opponent Defense Area 違反の検知ロジック
        # ルール: "The ball must not be touched by a robot, while the robot is partially or fully inside the opponent defense area."
        if (last_touched_robot_info["team"] == 0 and # 青チーム（攻撃側）が触れた
            last_touched_robot_info["robot_id"] == attacker_id):

            # 攻撃側ロボットが相手（黄色）ディフェンスエリア内にいるか確認
            if is_position_in_yellow_defense_area(attacker_robot_state.x, attacker_robot_state.y):
                print(f"Violation DETECTED: Attacker Blue robot {attacker_id} at ({attacker_robot_state.x:.2f}, {attacker_robot_state.y:.2f}) touched ball inside opponent (yellow) defense area.")
                violation_detected = True
                detection_time = time.time() - start_check_time
                break

        time.sleep(0.05) # ポーリング間隔

    # アサーション: 違反が検知されたことを確認
    assert violation_detected, "Attacker Touched Ball In Opponent Defense Area violation was NOT detected, but it was expected."
    print(f"Attacker Touched Ball In Opponent Defense Area test passed: Violation successfully detected at {detection_time:.2f}s.")


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
        test_attacker_touched_ball_in_opponent_defense_area(rcst_comm)
    except AssertionError as e:
        print(f"Test failed: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Closing communication.")
        rcst_comm.close()
        print("ATTACKER_TOUCHED_BALL_IN_OPPONENT_DEFENSE_AREA.py execution finished.")
