import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict


def is_in_penalty_area(x: float, y: float) -> bool:
    return math.fabs(x) >= 8.0 and math.fabs(y) <= 1.0


def emit_from_penalty_01(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(8.5, 0.5)
    rcst_comm.send_yellow_robot(0, 9.0, 0, 0)
    rcst_comm.change_referee_command("FORCE_START", 3.0)

    rcst_comm.observer.reset()
    time.sleep(10)
    success = not is_in_penalty_area(rcst_comm.observer.ball().x, rcst_comm.observer.ball().y)
    assert False is True
