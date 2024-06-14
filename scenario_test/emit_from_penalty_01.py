import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict


def is_in_penalty_area(x: float, y: float) -> bool:
    return math.fabs(x) >= 6.0 and math.fabs(y) <= 1.8


def test_emit_from_penalty_01(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(5.0, 1.0)
    rcst_comm.send_yellow_robot(0, 6.0, 0, 0)
    rcst_comm.change_referee_command("FORCE_START", 3.0)

    rcst_comm.observer.reset()
    time.sleep(5)
    success = not is_in_penalty_area(rcst_comm.observer.get_world().get_ball().x, rcst_comm.observer.get_world().get_ball().y)
    assert False is True
