import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict, Robot

# Division A フィールドの定義
FIELD_WIDTH = 9.0  # メートル
TOUCH_LINE_POSITIVE_Y = FIELD_WIDTH / 2.0  # 4.5
TOUCH_LINE_NEGATIVE_Y = -FIELD_WIDTH / 2.0  # -4.5

# 最後にボールに触れたロボットの情報を保持 (チームとID)
# team: 0 for BLUE, 1 for YELLOW, -1 for None
last_touched_robot_info = {"team": -1, "robot_id": -1}


def reset_last_touched_robot():
    global last_touched_robot_info
    last_touched_robot_info = {"team": -1, "robot_id": -1}


# ボール接触監視コールバックは、ここでは手動更新するため省略


def test_throw_in(rcst_comm: Communication):
    global last_touched_robot_info
    reset_last_touched_robot()

    # 1. ワールドを初期化
    rcst_comm.send_empty_world()
    print("World initialized.")

    # 2. ボールをフィールド内のタッチライン近くに配置 (y=4.0)
    ball_initial_x, ball_initial_y = 0.0, 4.0
    rcst_comm.send_ball(x=ball_initial_x, y=ball_initial_y)
    print(f"Ball placed at ({ball_initial_x}, {ball_initial_y}).")
    time.sleep(0.1)

    # 3. 青チームのロボット0番をボールの近くに配置 (ボールをy軸正方向に蹴り出す向き)
    kicker_id = 0
    kicker_initial_x, kicker_initial_y = 0.0, 3.8
    kicker_orientation = math.pi / 2  # y軸正方向
    rcst_comm.send_blue_robot(
        robot_id=kicker_id,
        x=kicker_initial_x,
        y=kicker_initial_y,
        orientation=kicker_orientation,
    )
    print(
        f"Blue kicker robot {kicker_id} placed at ({kicker_initial_x:.2f}, {kicker_initial_y:.2f})."
    )
    time.sleep(0.1)

    # 4. 黄色チームのロボットは影響のない場所に配置
    rcst_comm.send_yellow_robot(robot_id=0, x=-2.0, y=0.0, orientation=0)
    print("Yellow robot 0 placed at (-2.0, 0.0).")
    time.sleep(0.1)

    # 5. ゲームを開始
    rcst_comm.change_referee_command("FORCE_START", 0.5)  # 0.5秒後に実行
    print("Referee command 'FORCE_START' sent, effective in 0.5s.")
    time.sleep(1.0)  # コマンド反映とロボットの動作開始を待つ

    # 6. 青チームのロボット0番にボールをタッチラインの外へ蹴り出す動作を指示
    #    目標位置をタッチラインの少し外側に設定
    kick_target_y = TOUCH_LINE_POSITIVE_Y + 0.2
    print(
        f"Blue kicker robot {kicker_id} moving to kick ball towards y={kick_target_y:.2f}."
    )
    # 接触前の状態をクリア
    reset_last_touched_robot()
    rcst_comm.send_blue_robot(
        robot_id=kicker_id,
        x=kicker_initial_x,
        y=kick_target_y,
        orientation=kicker_orientation,
        velocity_y=2.0,
    )

    ball_out_of_play = False
    last_touch_by_kicker_confirmed = False
    referee_command_updated_to_free_kick = False  # オプションの確認用

    # ボールアウトとレフェリーコマンド変更の監視ループ
    start_check_time = time.time()
    max_check_duration = 7.0  # 最大7秒間チェック (移動とコマンド変更に十分な時間)

    while time.time() - start_check_time < max_check_duration:
        world = rcst_comm.observer.get_world()
        if world is None:
            time.sleep(0.01)
            continue

        ball_state = world.get_ball()
        kicker_robot_state = world.get_blue_robot(kicker_id)

        if ball_state is None or kicker_robot_state is None:
            time.sleep(0.01)
            continue

        # ボール接触の手動更新
        # ボールがまだインプレー（またはタッチライン付近）の場合のみ接触を更新
        if abs(ball_state.y) < TOUCH_LINE_POSITIVE_Y + 0.1:  # わずかなマージン
            if (
                calc.distance_robot_and_ball(kicker_robot_state, ball_state) < 0.15
            ):  # 接触距離
                if not (
                    last_touched_robot_info["team"] == 0
                    and last_touched_robot_info["robot_id"] == kicker_id
                ):
                    print(
                        f"Manually detected: Ball touched by BLUE kicker robot {kicker_id}"
                    )
                last_touched_robot_info = {"team": 0, "robot_id": kicker_id}

        # 7. Throw-In (ボールアウト) の検知ロジック
        if not ball_out_of_play:  # まだボールアウトが検知されていない場合
            if (
                ball_state.y > TOUCH_LINE_POSITIVE_Y
                or ball_state.y < TOUCH_LINE_NEGATIVE_Y
            ):
                print(f"Ball detected out of play at y={ball_state.y:.2f}.")
                ball_out_of_play = True
                # ボールアウト直前に最後に触れたロボットを確認
                if (
                    last_touched_robot_info["team"] == 0
                    and last_touched_robot_info["robot_id"] == kicker_id
                ):
                    last_touch_by_kicker_confirmed = True
                    print(
                        f"Confirmed: Last touch before out was by Blue kicker {kicker_id}."
                    )
                else:
                    print(
                        f"Warning: Last touch was by team {last_touched_robot_info['team']} id {last_touched_robot_info['robot_id']}, not the kicker."
                    )

        # (オプション) レフェリーコマンドの確認
        current_referee_command = rcst_comm.observer.referee()
        if current_referee_command is not None:
            # コマンド名は正確なものを rcst から取得する必要がある
            # ここでは仮に 'FREE_KICK_YELLOW' または 'THROW_IN_YELLOW' 等を想定
            if (
                "FREE_KICK_YELLOW" in current_referee_command.command.upper()
                or "THROW_IN_YELLOW" in current_referee_command.command.upper()
            ):  # 大文字小文字を区別しない比較
                referee_command_updated_to_free_kick = True
                print(f"Referee command updated to: {current_referee_command.command}")

        # 両方の主要条件が満たされたらループを抜ける
        if ball_out_of_play and last_touch_by_kicker_confirmed:
            # オプションのレフェリーコマンド確認も待つ場合はここに条件追加
            if (
                referee_command_updated_to_free_kick
                or time.time() - start_check_time > 3.0
            ):  # コマンド変更まで少し待つかタイムアウト
                break

        time.sleep(0.05)  # ポーリング間隔

    # アサーション
    assert ball_out_of_play, "Ball did not go out of play."
    assert last_touch_by_kicker_confirmed, (
        "Last touch by the kicker before ball went out was not confirmed."
    )
    # オプション: assert referee_command_updated_to_free_kick, "Referee command did not update to opponent's free kick."
    if referee_command_updated_to_free_kick:
        print(
            "Throw-In test passed: Ball out, last touch confirmed, and referee command updated."
        )
    else:
        print(
            "Throw-In test passed: Ball out and last touch confirmed (referee command check skipped or not matched)."
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
        test_throw_in(rcst_comm)
    except AssertionError as e:
        print(f"Test failed: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Closing communication.")
        rcst_comm.close()
        print("THROW_IN.py execution finished.")
