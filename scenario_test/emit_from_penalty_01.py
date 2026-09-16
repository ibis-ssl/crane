"""ペナルティエリア内のボールを外へ出すシナリオテスト。

ペナルティエリアの範囲は vision の geometry から導出する。以前は
`|x| >= 6.0 and |y| <= 1.8` と Division A のゴールライン座標を直書きしており、
フィールド内の点はどれもこの条件を満たさなかった（x=6.0 はゴールラインそのもの）。
つまり「ボールはペナルティエリア外にある」という判定が crane の振る舞いに
関係なく常に真で、このテストは何も検証していなかった。
"""

import time

from field_helpers import Field

# ボールをペナルティエリア前縁からどれだけ奥に置くか [m]
BALL_DEPTH_IN_PENALTY_AREA = 0.8
# ボールの y をペナルティエリア半幅に対してどの割合に置くか
BALL_Y_RATIO_IN_PENALTY_AREA = 0.55


def test_emit_from_penalty_01(field: Field):
    field.send_empty_world()

    ball_x = field.penalty_front_x(+1) + BALL_DEPTH_IN_PENALTY_AREA
    ball_y = field.penalty_half_width * BALL_Y_RATIO_IN_PENALTY_AREA
    assert field.is_in_penalty_area(ball_x, ball_y), (
        f"ボール ({ball_x:.3f}, {ball_y:.3f}) がペナルティエリア内にない。"
        f"前縁 x={field.penalty_front_x(+1):.3f}, 半幅 {field.penalty_half_width:.3f}"
    )

    field.send_ball(ball_x, ball_y)
    field.send_yellow_robot(0, field.from_goal_line(+1), 0, 0)
    field.comm.change_referee_command("FORCE_START", 3.0)

    field.comm.observer.reset()
    time.sleep(5)
    ball = field.comm.observer.get_world().get_ball()
    print(
        f"Ball: ({ball.x:.3f}, {ball.y:.3f}), "
        f"penalty front x={field.penalty_front_x(+1):.3f}, "
        f"half width={field.penalty_half_width:.3f}"
    )
    assert not field.is_in_penalty_area(ball.x, ball.y), (
        f"ボールがペナルティエリア内 ({ball.x:.3f}, {ball.y:.3f}) に残っている"
    )
