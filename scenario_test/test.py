from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict

def test(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(0, 0)
    for i in range(11):
        rcst_comm.send_blue_robot(i, -1.0, 3.0 - i * 0.5, 0)
    rcst_comm.change_referee_command('STOP', 3.0)
    rcst_comm.observer.reset()
    success = True
    assert True
