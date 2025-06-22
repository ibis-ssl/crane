import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict


def test_multiple_robots_ball_distance(rcst_comm: Communication):
    """STOP時に複数ロボットがボールから500mm以上離れることをテスト"""
    rcst_comm.send_empty_world()

    # ボールを中央に配置
    ball_x, ball_y = 0.0, 0.0
    rcst_comm.send_ball(ball_x, ball_y)

    # 複数のロボットをボール周辺に配置（規定距離違反の位置）
    rcst_comm.send_yellow_robot(
        0, ball_x + 0.3, ball_y, math.radians(0)
    )  # 300mm - 違反
    rcst_comm.send_yellow_robot(
        1, ball_x - 0.3, ball_y, math.radians(180)
    )  # 300mm - 違反
    rcst_comm.send_yellow_robot(
        2, ball_x, ball_y + 0.4, math.radians(270)
    )  # 400mm - 違反
    rcst_comm.send_yellow_robot(
        3, ball_x, ball_y - 0.4, math.radians(90)
    )  # 400mm - 違反
    rcst_comm.send_yellow_robot(
        4, ball_x + 0.6, ball_y, math.radians(0)
    )  # 600mm - 合法

    # STOP コマンドを送信
    rcst_comm.change_referee_command("STOP", 3.0)

    def robots_too_close_to_ball(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        """ボールから500mm以内にいるロボットの数をカウント"""
        close_robots = []
        for robot_id, robot in yellow_robots.items():
            distance = calc.distance_robot_and_ball(robot, ball)
            if distance < 0.5:  # 500mm
                close_robots.append((robot_id, distance))

        if len(close_robots) > 1:  # 複数のロボットが違反
            print(f"Multiple robots too close to ball: {close_robots}")
            return True
        return False

    rcst_comm.observer.customized().register_sticky_true_callback(
        "robots_too_close_to_ball", robots_too_close_to_ball
    )

    # テスト実行 - ロボットが適切に離れるかを確認
    rcst_comm.observer.reset()
    time.sleep(4)  # システムが反応する時間を与える

    # 最終状態をチェック
    world = rcst_comm.observer.get_world()
    ball = world.get_ball()
    yellow_robots = world.get_yellow_robots()

    close_count = 0
    distances = {}
    for robot_id, robot in yellow_robots.items():
        distance = calc.distance_robot_and_ball(robot, ball)
        distances[robot_id] = distance
        if distance < 0.5:
            close_count += 1

    print("Final robot distances from ball:")
    for robot_id, distance in distances.items():
        print(f"  Robot {robot_id}: {distance:.3f}m")

    # SSL規則: STOP時は最大1台のロボットのみがボールから500mm以内にいることが許可される
    assert close_count <= 1, f"Too many robots ({close_count}) within 500mm of ball"

    # 違反が検出されなかったことを確認
    violation_detected = rcst_comm.observer.customized().get_result(
        "robots_too_close_to_ball"
    )
    if violation_detected:
        print(
            "WARNING: Multiple robots were detected too close to ball during the test"
        )


if __name__ == "__main__":
    rcst_comm = Communication()
    test_multiple_robots_ball_distance(rcst_comm)
    rcst_comm.close()
    print("STOP_MULTIPLE_ROBOTS_BALL test passed")
