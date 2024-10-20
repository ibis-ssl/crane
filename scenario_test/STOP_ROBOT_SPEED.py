import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict


def test_robot_speed(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    ball_x = 1.0
    rcst_comm.send_ball(ball_x, 0)
    for i in range(11):
        rcst_comm.send_yellow_robot(i, -1.0, 3.0 - i * 0.5, math.radians(0))

    rcst_comm.change_referee_command("STOP", 3.0)

    def yellow_robot_did_not_move(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        for i in range(11):
            robot = yellow_robots[i]
            if not calc.distance(robot.x, robot.y, -1.0, 3.0 - i * 0.5) < 0.1:
                return False
        return True

    rcst_comm.observer.customized().register_sticky_true_callback(
        "yellow_robot_did_not_move", yellow_robot_did_not_move
    )

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

    assert rcst_comm.observer.customized().get_result("yellow_robot_did_not_move") is False


if __name__ == "__main__":
    rcst_comm = Communication()
    test_robot_speed(rcst_comm)
    rcst_comm.close()
    print("STOP_ROBOT_SPEED test passed")
