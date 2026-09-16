"""自陣ペナルティエリア内のボールを外へ出すシナリオテスト。

ペナルティエリアの範囲は vision の geometry から導出する。以前は
`|x| >= 6.0 and |y| <= 1.8` と Division A のゴールライン座標を直書きしており、
フィールド内の点はどれもこの条件を満たさなかった（x=6.0 はゴールラインそのもの）。
つまり「ボールはペナルティエリア外にある」という判定が crane の振る舞いに
関係なく常に真で、このテストは何も検証していなかった。

あわせてボールを置く側も直した。以前は +x 側、つまり crane から見て相手陣の
ペナルティエリアに置いており、そこのボールを片付けるのは誰の仕事でもない。
実測でも、ロボットを +x 側に置いても自陣（-x 側）のゴール前へ帰るだけで
ボールは 8 秒間 1mm も動かなかった。守る側は field_helpers.DEFENDED_SIDE。
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

    # 配置が効いたことを vision で確かめてから始める。このテストは長いあいだ
    # 「ボールがペナルティエリア外」という条件が常に真で何も検証していなかったので、
    # 出発点が本当にエリア内であることを前提条件として明示する。
    time.sleep(1.0)
    placed = field.comm.observer.get_world().get_ball()
    assert field.is_in_penalty_area(placed.x, placed.y), (
        f"開始時点でボール ({placed.x:.3f}, {placed.y:.3f}) が自陣ペナルティエリアに"
        f"入っていない。要求位置は ({ball_x:.3f}, {ball_y:.3f})"
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

    ball = field.comm.observer.get_world().get_ball()
    raise AssertionError(
        f"{EMIT_TIMEOUT:.0f}秒たってもボールが自陣ペナルティエリア内 "
        f"({ball.x:.3f}, {ball.y:.3f}) に残っている。"
        f"要求位置は ({ball_x:.3f}, {ball_y:.3f})、"
        f"前縁 x={front_x:.3f}, 半幅 {field.penalty_half_width:.3f}"
    )
