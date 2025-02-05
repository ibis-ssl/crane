# Teleport the ball to a new location and optionally set it to some velocity

# x-coordinate [m]
float32[<=1] x
# y-coordinate [m]
float32[<=1] y
# z-coordinate (height) [m]
float32[<=1] z
# Velocity in x-direction [m/s]
float32[<=1] vx
# Velocity in y-direction [m/s]
float32[<=1] vy
# Velocity in z-direction [m/s]
float32[<=1] vz
# Teleport the ball safely to the target, for example by
# moving robots out of the way in case of collision and set speed of robots close-by to zero default:false
bool[<=1] teleport_safely
# Adapt the angular ball velocity such that the ball is rolling default: false
bool[<=1] roll
# Instead of teleporting the ball, apply some force to make sure
# the ball reaches the required position soon (velocity is ignored if true)
# WARNING: A command with by_force stays active (the move will take some time)
# until cancled by another TeleportBall command with by_force = false.
# To avoid teleporting the ball at the end and resetting its current spin,
# do not set any of the optional fields in this message to end the force without triggering
# an additional teleportation [ default = false]
bool[<=1] by_force
