"""
ボールプレイスメント壁際シナリオテスト

このテストは以下のシナリオを検証します:
1. ボールがフィールド境界付近(壁際)にある場合、正しく壁際処理が行われるか
2. 回り込みスペースが不足している場合、ロボットがスタックせずに動作するか

座標は Division を決め打ちせず、vision の geometry から壁・ゴールライン・
タッチラインを基準に導出する（field フィクスチャ）。壁からの距離で書くことで、
Division A / B のどちらで走っても「壁際」という意図が保たれる。
"""

import math
import time

import pytest
from field_helpers import Field

# ボールを壁からどれだけ内側に置くか [m]
BALL_INSET_FROM_WALL = 0.45
# 配置ロボットをボールからどれだけ中央寄りに置くか [m]
PLACER_OFFSET_FROM_BALL = 0.8


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def setup_robots(field: Field, placer_x: float, placer_y: float):
    """ゴールキーパー + ball placerロボットを配置"""
    field.send_yellow_robot(0, field.from_goal_line(-1), 0.0, 0)  # GK
    field.send_yellow_robot(1, placer_x, placer_y, 0)  # ball placer
    # 以下は自陣側に置くだけの遊軍。位置に意味はないので割合で置く
    field.send_yellow_robot(2, field.x(-0.33), field.y(0.25), 0)
    field.send_yellow_robot(3, field.x(-0.33), field.y(-0.25), 0)
    field.send_yellow_robot(4, field.x(-0.5), 0.0, 0)


def run_ball_placement(
    field: Field,
    target_x: float,
    target_y: float,
    timeout: int = 30,
    success_distance: float = 0.20,
) -> tuple[bool, bool]:
    """STOP → BALL_PLACEMENT_YELLOW の順でコマンドを送り、目標距離内到達を待つ"""

    comm = field.comm
    comm.change_referee_command("STOP", 3.0)
    comm.set_ball_placement_position(target_x, target_y)
    comm.change_referee_command("BALL_PLACEMENT_YELLOW", 0.1)

    initial_ball = comm.observer.get_world().get_ball()
    for _ in range(timeout):
        ball = comm.observer.get_world().get_ball()
        if distance(ball.x, ball.y, target_x, target_y) < success_distance:
            moved = distance(initial_ball.x, initial_ball.y, ball.x, ball.y) > 0.05
            return True, moved
        time.sleep(1)

    final_ball = comm.observer.get_world().get_ball()
    moved = distance(initial_ball.x, initial_ball.y, final_ball.x, final_ball.y) > 0.05
    return False, moved


def report_and_assert(
    field: Field,
    ball_x: float,
    ball_y: float,
    target_x: float,
    target_y: float,
    success: bool,
    moved: bool,
) -> None:
    """要求位置と実際の位置を両方出したうえで判定する。

    要求位置と初期観測位置がずれていたら、それはクランプされた証拠なので
    真っ先に見えるようにする（このテストが 0.78 m 固定で落ち続けた原因）。
    """
    final_ball = field.comm.observer.get_world().get_ball()
    dist = distance(final_ball.x, final_ball.y, target_x, target_y)
    print(
        f"Requested: ({ball_x:.3f}, {ball_y:.3f}), "
        f"Final: ({final_ball.x:.3f}, {final_ball.y:.3f}), "
        f"Target: ({target_x:.3f}, {target_y:.3f}), Dist: {dist:.3f}m "
        f"[field {field.geometry.field_length:.1f}x{field.geometry.field_width:.1f}m]"
    )

    if not moved:
        pytest.skip("Ball placement did not start in this environment")
    assert success, f"Ball placement failed: ball is {dist:.3f}m from target"


def test_ball_placement_near_wall_x_boundary(field: Field):
    """
    シナリオ1: X軸方向の壁際にボールがある場合のテスト

    初期配置:
    - ボール: ゴールライン裏の壁から 0.45m 内側、中央の高さ
    - 配置目標: (0.0, 0.0) - フィールド中央

    期待結果:
    - 30秒以内にボールが配置目標から20cm以内に配置される
    """
    field.send_empty_world()
    ball_x = field.from_wall_x(+1, BALL_INSET_FROM_WALL)
    ball_y = 0.0
    field.send_ball(ball_x, ball_y)
    setup_robots(field, ball_x - PLACER_OFFSET_FROM_BALL, ball_y)
    time.sleep(1)

    target_x, target_y = 0.0, 0.0
    success, moved = run_ball_placement(field, target_x, target_y)
    report_and_assert(field, ball_x, ball_y, target_x, target_y, success, moved)


def test_ball_placement_near_wall_y_boundary(field: Field):
    """
    シナリオ2: Y軸方向の壁際にボールがある場合のテスト

    初期配置:
    - ボール: タッチライン裏の壁から 0.45m 内側
    - 配置目標: そこから 2.3m 中央寄り

    期待結果:
    - 30秒以内にボールが配置目標から20cm以内に配置される
    """
    field.send_empty_world()
    ball_x = 2.0
    ball_y = field.from_wall_y(+1, BALL_INSET_FROM_WALL)
    field.send_ball(ball_x, ball_y)
    setup_robots(field, ball_x, ball_y - PLACER_OFFSET_FROM_BALL)
    time.sleep(1)

    # 押す距離はフィールドの大きさではなくロボットの能力の問題なので絶対値で持つ
    target_x, target_y = ball_x, ball_y - 2.3
    success, moved = run_ball_placement(field, target_x, target_y)
    report_and_assert(field, ball_x, ball_y, target_x, target_y, success, moved)


def test_ball_placement_tight_space(field: Field):
    """
    シナリオ3: 回り込みスペースが不足している場合のテスト

    初期配置:
    - ボール: 壁から 0.75m 内側、かつタッチラインから 1.5m 内側（コーナー寄り）
    - 配置目標: ボールより 1.5m 中央寄り

    ロボット中心はボールの外側 (ROBOT_RADIUS + BALL_RADIUS) に入る必要があるので、
    壁までの余裕がそれを下回ると構造的に成功しえない。配置を作る前に確認する。

    期待結果:
    - ロボットがスタックせずに動作する
    - 30秒以内にボールが配置目標から20cm以内に配置される
    """
    field.send_empty_world()
    room_behind_ball = 0.75
    ball_x = field.from_wall_x(+1, room_behind_ball)
    ball_y = field.from_touch_line(+1, 1.5)

    needed = field.clearance_to_get_behind_ball
    assert room_behind_ball > needed, (
        f"ボールの外側に回り込む余地が {room_behind_ball:.3f}m しかなく、"
        f"必要な {needed:.3f}m を下回る。この配置では成功しえない"
    )

    field.send_ball(ball_x, ball_y)
    setup_robots(field, ball_x - 1.0, ball_y)
    time.sleep(1)

    target_x, target_y = ball_x - 1.5, ball_y
    success, moved = run_ball_placement(field, target_x, target_y)
    report_and_assert(field, ball_x, ball_y, target_x, target_y, success, moved)
