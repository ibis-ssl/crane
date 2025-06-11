import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict, Robot

# Division A フィールドとディフェンスエリアの定義
FIELD_LENGTH = 12.0  # メートル
FIELD_WIDTH = 9.0  # メートル
DEFENSE_AREA_LENGTH = 1.8  # X軸方向の長さ (ゴールラインからの奥行き)
DEFENSE_AREA_WIDTH = 3.6  # Y軸方向の幅

# 青チームのゴールがX軸負側にあると仮定
BLUE_GOAL_X = -FIELD_LENGTH / 2.0  # -6.0
BLUE_DEFENSE_AREA_X_MIN = BLUE_GOAL_X  # -6.0
BLUE_DEFENSE_AREA_X_MAX = BLUE_GOAL_X + DEFENSE_AREA_LENGTH  # -4.2
BLUE_DEFENSE_AREA_Y_MIN = -DEFENSE_AREA_WIDTH / 2.0  # -1.8
BLUE_DEFENSE_AREA_Y_MAX = DEFENSE_AREA_WIDTH / 2.0  #  1.8

# 最後にボールに触れたロボットの情報を保持 (チームとID)
# team: 0 for BLUE, 1 for YELLOW, -1 for None
last_touched_robot_info = {"team": -1, "robot_id": -1}


def reset_last_touched_robot():
    global last_touched_robot_info
    last_touched_robot_info = {"team": -1, "robot_id": -1}


def is_position_in_blue_defense_area(x: float, y: float) -> bool:
    """指定された座標が青チームのディフェンスエリア内にあるかを判定する"""
    return (
        BLUE_DEFENSE_AREA_X_MIN <= x <= BLUE_DEFENSE_AREA_X_MAX
        and BLUE_DEFENSE_AREA_Y_MIN <= y <= BLUE_DEFENSE_AREA_Y_MAX
    )


def ball_touch_monitor_callback(
    ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
):
    """
    ボールに最後に触れたロボットを記録するコールバック。
    単純な距離ベースの接触判定。
    """
    global last_touched_robot_info
    # 簡易的な接触判定距離
    CONTACT_DISTANCE_THRESHOLD = 0.15  # ロボット半径 + ボール半径程度

    for robot_id, robot in blue_robots.items():
        if calc.distance_robot_and_ball(robot, ball) < CONTACT_DISTANCE_THRESHOLD:
            if (
                last_touched_robot_info["team"] != 0
                or last_touched_robot_info["robot_id"] != robot_id
            ):
                print(f"[Callback] Ball touched by BLUE robot {robot_id}")
            last_touched_robot_info = {"team": 0, "robot_id": robot_id}
            return

    for robot_id, robot in yellow_robots.items():
        if calc.distance_robot_and_ball(robot, ball) < CONTACT_DISTANCE_THRESHOLD:
            if (
                last_touched_robot_info["team"] != 1
                or last_touched_robot_info["robot_id"] != robot_id
            ):
                print(f"[Callback] Ball touched by YELLOW robot {robot_id}")
            last_touched_robot_info = {"team": 1, "robot_id": robot_id}
            return
    # 誰も触れていなければ更新しない


def test_multiple_defenders(rcst_comm: Communication):
    global last_touched_robot_info
    reset_last_touched_robot()

    # 1. ワールドを初期化
    rcst_comm.send_empty_world()
    print("World initialized.")

    # 2. 青チームのディフェンスエリア内にボールを配置
    ball_x, ball_y = -5.0, 0.0  # ディフェンスエリア内 (-6 to -4.2, -1.8 to 1.8)
    assert is_position_in_blue_defense_area(ball_x, ball_y), (
        "Ball initial position is not in defense area!"
    )
    rcst_comm.send_ball(x=ball_x, y=ball_y)
    print(f"Ball placed at ({ball_x}, {ball_y}) in blue defense area.")
    time.sleep(0.1)

    # 3. 青チームのロボット1番（キーパーではない）をディフェンスエリア内でボールに触れられる位置に配置
    non_keeper_id = 1
    non_keeper_x, non_keeper_y = -5.1, 0.0  # ボールのすぐ近く
    non_keeper_orientation = 0  # ボール方向 (X軸正)
    assert is_position_in_blue_defense_area(non_keeper_x, non_keeper_y), (
        "Non-keeper initial position is not in defense area!"
    )
    rcst_comm.send_blue_robot(
        robot_id=non_keeper_id,
        x=non_keeper_x,
        y=non_keeper_y,
        orientation=non_keeper_orientation,
    )
    print(
        f"Blue non-keeper robot {non_keeper_id} placed at ({non_keeper_x:.2f}, {non_keeper_y:.2f})."
    )
    time.sleep(0.1)

    # 4. 青チームのキーパーロボット（ロボット0番）をディフェンスエリア内の別の場所に配置
    keeper_id = 0
    keeper_x, keeper_y = -5.9, 0.1  # ゴールライン近く
    keeper_orientation = 0
    assert is_position_in_blue_defense_area(keeper_x, keeper_y), (
        "Keeper initial position is not in defense area!"
    )
    rcst_comm.send_blue_robot(
        robot_id=keeper_id, x=keeper_x, y=keeper_y, orientation=keeper_orientation
    )
    print(f"Blue keeper robot {keeper_id} placed at ({keeper_x:.2f}, {keeper_y:.2f}).")
    time.sleep(0.1)

    # 5. 黄色チームのロボットはフィールド中央など、影響のない場所に配置
    rcst_comm.send_yellow_robot(robot_id=0, x=0.0, y=0.0, orientation=0)
    print("Yellow robot 0 placed at (0.0, 0.0).")
    time.sleep(0.1)

    # ボール接触監視コールバックの登録 (rcst.observer.customized() があれば)
    # ここでは手動で接触を確認するアプローチをとるためコメントアウト
    # rcst_comm.observer.customized().register_every_frame_callback("BALL_TOUCH_MONITOR", ball_touch_monitor_callback)

    # 6. ゲームを開始
    rcst_comm.change_referee_command("FORCE_START", 0.5)  # 0.5秒後に実行
    print("Referee command 'FORCE_START' sent, effective in 0.5s.")
    time.sleep(1.0)  # コマンド反映とロボットの動作開始を待つ

    # 7. 青チームのロボット1番がボールに触れるようにする
    #    ボールとロボット1が近いため、微小な移動か、そのままで接触を期待。
    #    明示的に接触させるために、ボール位置へ移動させる。
    print(f"Moving blue non-keeper robot {non_keeper_id} to touch the ball...")
    # 接触前のlast_touched_robot_infoをクリアしておく
    reset_last_touched_robot()
    rcst_comm.send_blue_robot(
        robot_id=non_keeper_id,
        x=ball_x,
        y=ball_y,
        orientation=non_keeper_orientation,
        velocity_x=0.5,
    )

    multiple_defenders_violation_detected = False
    detection_time = 0.0

    # 接触と違反判定のループ
    start_check_time = time.time()
    max_check_duration = 5.0  # 最大5秒間チェック

    while time.time() - start_check_time < max_check_duration:
        world = rcst_comm.observer.get_world()
        if world is None:
            time.sleep(0.01)
            continue

        ball_state = world.get_ball()
        non_keeper_robot_state = world.get_blue_robot(non_keeper_id)

        if ball_state is None or non_keeper_robot_state is None:
            time.sleep(0.01)
            continue

        # ボール接触の手動更新 (コールバックがない場合の代替)
        if (
            calc.distance_robot_and_ball(non_keeper_robot_state, ball_state) < 0.15
        ):  # 接触距離
            if not (
                last_touched_robot_info["team"] == 0
                and last_touched_robot_info["robot_id"] == non_keeper_id
            ):
                print(f"Manually detected: Ball touched by BLUE robot {non_keeper_id}")
            last_touched_robot_info = {"team": 0, "robot_id": non_keeper_id}

        # 8. Multiple Defenders違反の検知ロジック
        # ルール: "If a robot other than the keeper touches the ball while this robot is entirely inside its own defense area..."
        if (
            last_touched_robot_info["team"] == 0
            and last_touched_robot_info["robot_id"] == non_keeper_id
            and non_keeper_robot_state.id != keeper_id
        ):  # キーパーではないことを確認
            # ロボット1番がディフェンスエリア内にいるか確認
            if is_position_in_blue_defense_area(
                non_keeper_robot_state.x, non_keeper_robot_state.y
            ):
                print(
                    f"Violation DETECTED: Non-keeper Blue robot {non_keeper_id} touched ball at ({non_keeper_robot_state.x:.2f}, {non_keeper_robot_state.y:.2f}) which is inside the defense area."
                )
                multiple_defenders_violation_detected = True
                detection_time = time.time() - start_check_time
                break

        time.sleep(0.05)  # ポーリング間隔

    # アサーション: Multiple Defenders 違反が検知されたことを確認
    assert multiple_defenders_violation_detected, (
        "Multiple Defenders violation was NOT detected, but it was expected."
    )
    print(
        f"Multiple Defenders test passed: Violation successfully detected at {detection_time:.2f}s."
    )


if __name__ == "__main__":
    try:
        from rcst.communication import Communication
        from rcst import calc
        from rcst.ball import Ball
        from rcst.robot import RobotDict, Robot
    except ModuleNotFoundError:
        print(
            "Error: rcst library not found. Please ensure it is installed and accessible."
        )
        exit(1)

    rcst_comm = Communication()

    if not hasattr(rcst_comm, "send_blue_robot"):
        print(
            "Warning: rcst_comm.send_blue_robot does not exist. Using a dummy function."
        )

        def _dummy_send_blue_robot(
            robot_id: int, x: float, y: float, orientation: float, **kwargs
        ):
            print(
                f"DUMMY send_blue_robot: id={robot_id}, x={x}, y={y}, orientation={orientation}"
            )

        rcst_comm.send_blue_robot = _dummy_send_blue_robot

    if not hasattr(rcst_comm, "send_yellow_robot"):
        print(
            "Warning: rcst_comm.send_yellow_robot does not exist. Using a dummy function."
        )

        def _dummy_send_yellow_robot(
            robot_id: int, x: float, y: float, orientation: float, **kwargs
        ):
            print(
                f"DUMMY send_yellow_robot: id={robot_id}, x={x}, y={y}, orientation={orientation}"
            )

        rcst_comm.send_yellow_robot = _dummy_send_yellow_robot

    try:
        test_multiple_defenders(rcst_comm)
    except AssertionError as e:
        print(f"Test failed: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Closing communication.")
        rcst_comm.close()
        print("MULTIPLE_DEFENDERS.py execution finished.")
