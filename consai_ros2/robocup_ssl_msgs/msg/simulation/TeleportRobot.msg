# Teleport a robot to some location and give it a velocity

# Robot id to teleport
RobotId id
# x-coordinate [m]
float32[<=1] x
# y-coordinate [m]
float32[<=1] y
# Orientation [rad], measured from the x-axis counter-clockwise
float32[<=1] orientation
# Global velocity [m/s] towards x-axis
float32[<=1] v_x
# Global velocity [m/s] towards y-axis
float32[<=1] v_y
# Angular velocity [rad/s]
float32[<=1] v_angular
# Robot should be present on the field?
# true -> robot will be added, if it does not exist yet
# false -> robot will be removed, if it is present
bool[<=1] present
# Instead of teleporting, apply some force to make sure
# the robot reaches the required position soon (velocity is ignored if true)
# WARNING: A command with by_force stays active (the move will take some time)
# until cancled by another TeleportRobot command for the same bot with by_force = false.
# To avoid teleporting at the end,
# do not set any of the optional fields in this message
# to end the force without triggering
# an additional teleportation default : false
bool[<=1] by_force
