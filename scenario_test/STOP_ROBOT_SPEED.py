import math
import time
from rcst.communication import Communication
from rcst import calc


def test_robot_speed(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    ball_x = 1.0
    rcst_comm.send_ball(ball_x, 0)
    for i in range(11):
        rcst_comm.send_yellow_robot(i, -1.0, 3.0 - i * 0.5, math.radians(0))

    rcst_comm.change_referee_command("STOP", 3.0)

    rcst_comm.observer.reset()
    success = True
    rcst_comm.send_ball(ball_x, 0, 5.0, 0.0)  # Move the ball
    for _ in range(10):
        if rcst_comm.observer.robot_speed().some_yellow_robots_over(1.5):
            velocities = rcst_comm.observer.robot_speed().yellow_max_velocities()
            for robot_id in velocities.keys():
                print(f"Robot {robot_id} has speed {velocities[robot_id]}")
            success = False
            break
        time.sleep(1)
    assert success is True

    # 10秒後の最終位置で判定: 少なくとも1台のロボットが初期位置から0.1m以上移動しているか
    some_robot_moved = False
    for i in range(11):
        robot = rcst_comm.observer.get_world().get_yellow_robot(i)
        dist = calc.distance(robot.x, robot.y, -1.0, 3.0 - i * 0.5)
        if dist >= 0.1:
            print(f"Robot {i} moved {dist:.3f}m from initial position")
            some_robot_moved = True
    assert some_robot_moved, (
        "No yellow robot moved during STOP (expected at least one to reposition)"
    )


if __name__ == "__main__":
    rcst_comm = Communication()
    test_robot_speed(rcst_comm)
    rcst_comm.close()
    print("STOP_ROBOT_SPEED test passed")
