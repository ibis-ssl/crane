"""自陣ペナルティエリア内のボールを外へ出すシナリオテスト。

ペナルティエリアの範囲は vision の geometry から導出する。座標を直書きすると
区分が違うだけで「エリア外」の判定が常に真になり、何も検証しなくなる。

ボールは守る側（field_helpers.DEFENDED_SIDE）のエリアに置く。相手陣のエリアの
ボールを片付けるのは誰の仕事でもなく、crane は自陣へ帰るだけで動かさない。
"""

import time

from field_helpers import DEFENDED_SIDE, Field

# ボールをペナルティエリア前縁からどれだけ奥に置くか [m]
BALL_DEPTH_IN_PENALTY_AREA = 0.8
# ボールの y をペナルティエリア半幅に対してどの割合に置くか
BALL_Y_RATIO_IN_PENALTY_AREA = 0.55
# ボールが出るのを待つ時間 [s]
EMIT_TIMEOUT = 15.0


def test_emit_from_penalty_01(field: Field):
    field.send_empty_world()

    front_x = field.penalty_front_x(DEFENDED_SIDE)
    ball_x = front_x + DEFENDED_SIDE * BALL_DEPTH_IN_PENALTY_AREA
    ball_y = field.penalty_half_width * BALL_Y_RATIO_IN_PENALTY_AREA
    assert field.is_in_penalty_area(ball_x, ball_y), (
        f"ボール ({ball_x:.3f}, {ball_y:.3f}) が自陣ペナルティエリア内にない。"
        f"前縁 x={front_x:.3f}, 半幅 {field.penalty_half_width:.3f}"
    )

    field.send_ball(ball_x, ball_y)
    field.send_yellow_robot(0, field.from_goal_line(DEFENDED_SIDE, 0.3), 0, 0)

    # 配置が効いたことを vision で確かめてから始める。出発点がエリア外だと
    # 下の「エリアを出た」判定が即成立し、何も検証しないまま通ってしまう。
    time.sleep(1.0)
    placed = field.comm.observer.get_world().get_ball()
    assert field.is_in_penalty_area(placed.x, placed.y), (
        f"開始時点でボール ({placed.x:.3f}, {placed.y:.3f}) が自陣ペナルティエリアに"
        f"入っていない。要求位置は ({ball_x:.3f}, {ball_y:.3f})"
    )
    # is_in_penalty_area は |x| で見るので左右を区別しない。守る側が反転していると
    # 相手ペナルティエリアに置いたまま上の assert を通過し、タイムアウトまで
    # 「出ない」と報告されて原因が見えなくなる。側そのものを明示的に確かめる。
    assert placed.x * DEFENDED_SIDE > 0, (
        f"ボール ({placed.x:.3f}, {placed.y:.3f}) が守る側 "
        f"(x の符号 {DEFENDED_SIDE:+d}) にない。"
        f"DEFENDED_SIDE の前提が崩れている可能性がある（field_helpers.py の説明を参照）"
    )

    field.comm.change_referee_command("FORCE_START", 3.0)

    field.comm.observer.reset()
    deadline = time.time() + EMIT_TIMEOUT
    while time.time() < deadline:
        ball = field.comm.observer.get_world().get_ball()
        if not field.is_in_penalty_area(ball.x, ball.y):
            print(f"ボールがペナルティエリアを出た: ({ball.x:.3f}, {ball.y:.3f})")
            return
        time.sleep(0.5)

    world = field.comm.observer.get_world()
    ball = world.get_ball()
    # ロボットが自陣へ帰ってボールを放置しているのか、寄っているのに出せないのかを
    # ログだけで切り分けられるようにロボット位置も出す。
    robots = ", ".join(
        f"Y{r.id}({r.x:.3f}, {r.y:.3f})" for r in world.get_yellow_robots().values()
    )
    raise AssertionError(
        f"{EMIT_TIMEOUT:.0f}秒たってもボールが自陣ペナルティエリア内 "
        f"({ball.x:.3f}, {ball.y:.3f}) に残っている。"
        f"要求位置は ({ball_x:.3f}, {ball_y:.3f})、"
        f"前縁 x={front_x:.3f}, 半幅 {field.penalty_half_width:.3f}、"
        f"yellow: {robots or 'なし'}"
    )
