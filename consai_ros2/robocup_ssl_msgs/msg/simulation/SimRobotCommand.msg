# Full command for a single robot

# Id of the robot
uint32 id
# Movement command
RobotMoveCommand[<=1] move_command
# Absolute (3 dimensional) kick speed [m/s]
float32[<=1] kick_speed
# Kick angle [degree] (defaults to 0 degrees for a straight kick)
float32[<=1] kick_angle
# Dribbler speed in rounds per minute [rpm]
float32[<=1] dribbler_speed
