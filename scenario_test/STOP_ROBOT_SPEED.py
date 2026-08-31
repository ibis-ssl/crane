import math
import time

from rcst.communication import Communication


def test_robot_speed(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    ball_x = 1.0
    rcst_comm.send_ball(ball_x, 0)
    for i in range(11):
        rcst_comm.send_yellow_robot(i, -1.0, 3.0 - i * 0.5, math.radians(0))

    rcst_comm.change_referee_command("STOP", 3.0)

    rcst_comm.observer.reset()
    rcst_comm.send_ball(ball_x, 0, 5.0, 0.0)  # Move the ball
    for _ in range(10):
        if rcst_comm.observer.robot_speed().some_yellow_robots_over(1.5):
            velocities = rcst_comm.observer.robot_speed().yellow_max_velocities()
            for robot_id in velocities.keys():
                print(f"Robot {robot_id} has speed {velocities[robot_id]}")
            assert False, "Yellow robot exceeded 1.5 m/s during STOP"
        time.sleep(1)


if __name__ == "__main__":
    rcst_comm = Communication()
    test_robot_speed(rcst_comm)
    rcst_comm.close()
    print("STOP_ROBOT_SPEED test passed")
