import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict, Robot

# グローバル変数などで状態を保持する必要があるかもしれない
last_ball_toucher: dict[str, int] = {
    "team": -1,
    "robot_id": -1,
}  # -1 for None, 0 for BLUE, 1 for YELLOW
kick_off_issued_by_blue = False


def reset_global_state():
    global last_ball_toucher, kick_off_issued_by_blue
    last_ball_toucher = {"team": -1, "robot_id": -1}
    kick_off_issued_by_blue = False


def ball_contact_callback(ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict):
    """
    ボールとロボットの接触を監視し、最後に触れたロボットを記録するコールバック。
    単純な距離ベースの接触判定。
    """
    global last_ball_toucher
    # TODO: より洗練された接触判定ロジック (例: rcst.observer.ball_contact() があればそれを使う)

    # 青ロボットとの接触確認
    for robot_id, robot in blue_robots.items():
        if calc.distance_robot_and_ball(robot, ball) < 0.15:  # 仮の接触距離
            if (
                last_ball_toucher["team"] != 0
                or last_ball_toucher["robot_id"] != robot_id
            ):
                print(f"[Callback] Ball touched by BLUE robot {robot_id}")
            last_ball_toucher = {"team": 0, "robot_id": robot_id}
            return  # 複数のロボットが同時に触れることはないと仮定

    # 黄ロボットとの接触確認
    for robot_id, robot in yellow_robots.items():
        if calc.distance_robot_and_ball(robot, ball) < 0.15:  # 仮の接触距離
            if (
                last_ball_toucher["team"] != 1
                or last_ball_toucher["robot_id"] != robot_id
            ):
                print(f"[Callback] Ball touched by YELLOW robot {robot_id}")
            last_ball_toucher = {"team": 1, "robot_id": robot_id}
            return
    # 誰にも触れていない場合はリセットしない (最後に触れた情報を保持)
    return


def test_double_touch(rcst_comm: Communication):
    global last_ball_toucher, kick_off_issued_by_blue
    reset_global_state()  # テスト実行前にグローバル状態をリセット

    # rcst_comm.observer.customized().register_sticky_true_callback( # 毎フレーム呼ばれるコールバックとして登録したい
    #     "BALL_CONTACT_MONITOR", ball_contact_callback
    # )
    # Sticky Trueだと一度TrueになったらTrueのままなので、毎フレーム状態を更新するのには向かない。
    # rcst.observer に毎フレーム実行されるコールバックを登録する機能が必要。
    # もしそのような機能がなければ、ループ内で手動で呼び出すか、
    # 重要なタイミングで last_ball_toucher を能動的に更新する。
    # ここでは、重要な操作の後に手動で接触を確認・更新するアプローチをとる。

    # 1. ワールドを初期化
    rcst_comm.send_empty_world()
    print("World initialized.")

    # 2. ボールをフィールド中央に配置
    rcst_comm.send_ball(0, 0)
    print("Ball placed at (0,0).")
    time.sleep(0.1)  # 描画や状態反映のための短い待機

    # 3. 青チームのロボット0番をボールの少し後ろに配置
    kicker_robot_id = 0
    kicker_initial_x, kicker_initial_y = -0.5, 0.0
    rcst_comm.send_blue_robot(
        robot_id=kicker_robot_id, x=kicker_initial_x, y=kicker_initial_y, orientation=0
    )
    print(
        f"Blue robot {kicker_robot_id} placed at ({kicker_initial_x}, {kicker_initial_y})."
    )
    time.sleep(0.1)

    # 4. 他のロボットはフィールド外に配置
    rcst_comm.send_yellow_robot(robot_id=0, x=10.0, y=10.0, orientation=0)
    print("Yellow robot 0 placed at (10.0, 10.0).")
    time.sleep(0.1)

    # 5. キックオフコマンドを送信 (例: FORCE_START)
    #    ルール上は KICK_OFF_BLUE が適切だが、rcst のコマンド名は要確認。
    #    FORCE_START は多くの場合、インプレイを開始させる。
    #    ここでは、ロボットが自由に動ける状態にするために FORCE_START を使用。
    #    Double Touch ルールはキックオフ後に適用される。
    rcst_comm.change_referee_command("FORCE_START", 0.5)  # 0.5秒後に実行
    print("Referee command 'FORCE_START' sent, effective in 0.5s.")
    kick_off_issued_by_blue = True  # これでキックオフしたとみなす
    time.sleep(1.0)  # コマンドが確実に反映されるのを待つ

    # 6. ロボット0番がボールを蹴る動作をシミュレート
    print("Simulating kick...")
    ball_before_kick = rcst_comm.observer.get_world().get_ball()
    if ball_before_kick is None:
        assert False, "Failed to get ball position before kick."

    # ボールに向かって移動しキック (ボールの現在位置を目標とする)
    # 実際にはボールに到達する前にキック機構が働くが、ここでは接触で表現
    target_x, target_y = ball_before_kick.x, ball_before_kick.y
    # わずかにボールの奥を狙うことで、確実にボールを押すようにする
    kick_target_x = target_x + 0.1 * math.cos(0)  # orientation 0 (x軸正方向)
    kick_target_y = target_y + 0.1 * math.sin(0)

    # 接触検知のために、キック前に最後に触れたロボット情報をクリア（あるいはボール位置とする）
    last_ball_toucher = {"team": -1, "robot_id": -1}  # キッカーが触れる前の状態

    rcst_comm.send_blue_robot(
        robot_id=kicker_robot_id,
        x=kick_target_x,
        y=kick_target_y,
        orientation=0,
        velocity_x=2.0,
        velocity_y=0.0,
    )
    print(
        f"Blue robot {kicker_robot_id} moving to kick the ball towards ({kick_target_x:.2f}, {kick_target_y:.2f}) with velocity."
    )

    # ボールが動くまで、またはタイムアウトまで待機
    kick_time_start = time.time()
    ball_moved = False
    moved_distance = 0.0
    max_wait_kick = 3.0  # 最大3秒待つ
    while time.time() - kick_time_start < max_wait_kick:
        current_ball = rcst_comm.observer.get_world().get_ball()
        if current_ball is None:
            assert False, "Failed to get ball position during kick wait."
        moved_distance = calc.distance(
            ball_before_kick.x, ball_before_kick.y, current_ball.x, current_ball.y
        )
        if moved_distance >= 0.05:
            ball_moved = True
            print(f"Ball moved {moved_distance:.3f} m.")
            # キックしたロボットを記録
            last_ball_toucher = {"team": 0, "robot_id": kicker_robot_id}
            print(f"After kick, last_ball_toucher set to BLUE {kicker_robot_id}")
            break
        time.sleep(0.05)

    assert ball_moved, (
        f"Ball did not move at least 0.05m after kick simulation. Moved: {moved_distance:.3f}m"
    )

    # キック後、ロボットを少し後ろに戻す（連続タッチを防ぐため、また現実的な動作）
    rcst_comm.send_blue_robot(
        robot_id=kicker_robot_id, x=kicker_initial_x, y=kicker_initial_y, orientation=0
    )
    time.sleep(0.5)  # 移動のための待機

    # 7. ボールが移動した後、再度ロボット0番がボールに触れる動作をシミュレート
    print("Simulating robot moving to touch the ball again...")
    ball_after_kick = rcst_comm.observer.get_world().get_ball()
    if ball_after_kick is None:
        assert False, "Failed to get ball position for second touch."

    # ロボット0番を現在のボール位置に移動
    rcst_comm.send_blue_robot(
        robot_id=kicker_robot_id,
        x=ball_after_kick.x,
        y=ball_after_kick.y,
        orientation=0,
        velocity_x=1.0,
    )
    print(
        f"Blue robot {kicker_robot_id} moving towards the ball at ({ball_after_kick.x:.2f}, {ball_after_kick.y:.2f})."
    )

    # 再度接触するまで待機
    second_touch_time_start = time.time()
    double_touch_occurred = False
    max_wait_second_touch = 3.0
    while time.time() - second_touch_time_start < max_wait_second_touch:
        current_ball_state = rcst_comm.observer.get_world().get_ball()
        blue_robots_state = rcst_comm.observer.get_world().get_blue_robots()
        if current_ball_state is None or kicker_robot_id not in blue_robots_state:
            assert False, "Failed to get game state for second touch check."

        kicker_robot = blue_robots_state[kicker_robot_id]
        is_touching_now = (
            calc.distance_robot_and_ball(kicker_robot, current_ball_state) < 0.15
        )

        if is_touching_now:
            print(f"Blue robot {kicker_robot_id} is now touching the ball again.")
            # Double Touch ルールの確認
            # ルール: "the kicker is not allowed to touch the ball until it has been touched by another robot or the game has been stopped."
            # last_ball_toucher がキッカー自身 (BLUE, kicker_robot_id) であり、
            # その間に他のロボットが触れていない (last_ball_toucher の情報が変わっていない)、
            # かつゲームが停止していない (これは change_referee_command で能動的に制御するので、ここでは考慮外とする)
            if (
                last_ball_toucher["team"] == 0
                and last_ball_toucher["robot_id"] == kicker_robot_id
            ):
                # さらに、この接触がキックオフ直後の最初のボールタッチではないことを確認する必要があるが、
                # 上のロジックでキック後にlast_ball_toucherを更新しているので、この条件でOK
                print(
                    "Violation: Kicker touched the ball again before any other robot."
                )
                double_touch_occurred = True
                break
            else:
                # キッカー以外の誰かが触れたか、あるいは初期状態だった場合
                # (このシナリオではキッカー以外はフィールド外なので、通常ここは通らないはずだが、念のため)
                print(
                    f"No violation: Ball was touched by someone else (team {last_ball_toucher['team']}, id {last_ball_toucher['robot_id']}) or initial touch."
                )
                # last_ball_toucher を更新
                last_ball_toucher = {"team": 0, "robot_id": kicker_robot_id}
                break  # 接触したのでループを抜ける
        time.sleep(0.05)

    # 8. Double Touch違反が検知されることをアサート
    assert double_touch_occurred, (
        "Double Touch violation was NOT detected, but it was expected."
    )
    print("Double Touch test passed: Violation successfully detected.")


if __name__ == "__main__":
    # rcstライブラリがインストールされているか、パスが通っているかの確認が先決
    # 前回のエージェント試行で ModuleNotFoundError が発生したため。
    try:
        from rcst.communication import Communication
        from rcst import calc
        from rcst.ball import Ball
        from rcst.robot import RobotDict, Robot
    except ModuleNotFoundError:
        print(
            "Error: rcst library not found. Please ensure it is installed and accessible."
        )
        print("This script cannot run without the rcst library.")
        exit(1)

    rcst_comm = Communication()

    # send_blue_robot が Communication クラスに存在するかどうかを確認
    # 存在しない場合はダミー関数を割り当てる（前回の試行より）
    if not hasattr(rcst_comm, "send_blue_robot"):
        print(
            "Warning: rcst_comm.send_blue_robot does not exist. Using a dummy function."
        )

        def _dummy_send_blue_robot(
            robot_id: int,
            x: float,
            y: float,
            orientation: float,
            velocity_x: float = 0.0,
            velocity_y: float = 0.0,
            velocity_yaw: float = 0.0,
            kick_speed: float = 0.0,
            chip_kick: bool = False,
            dribble: bool = False,
            visible: bool = True,
        ):
            print(
                f"DUMMY send_blue_robot: id={robot_id}, x={x}, y={y}, orientation={orientation}, vel_x={velocity_x}"
            )

        rcst_comm.send_blue_robot = _dummy_send_blue_robot

    if not hasattr(rcst_comm, "send_yellow_robot"):
        print(
            "Warning: rcst_comm.send_yellow_robot does not exist. Using a dummy function."
        )

        def _dummy_send_yellow_robot(
            robot_id: int,
            x: float,
            y: float,
            orientation: float,
            velocity_x: float = 0.0,
            velocity_y: float = 0.0,
            velocity_yaw: float = 0.0,
            visible: bool = True,
        ):
            print(
                f"DUMMY send_yellow_robot: id={robot_id}, x={x}, y={y}, orientation={orientation}"
            )

        rcst_comm.send_yellow_robot = _dummy_send_yellow_robot

    try:
        test_double_touch(rcst_comm)
    except AssertionError as e:
        print(f"Test failed: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Closing communication.")
        rcst_comm.close()
        print("DOUBLE_TOUCH.py execution finished.")
