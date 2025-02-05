# Specs of a robot

# Id of the robot
RobotId id
# Robot radius [m] 0.09
float32[<=1] radius
# Robot height [m] 0.15
float32[<=1] height
# Robot mass [kg]
float32[<=1] mass
# Max linear kick speed [m/s] (unset = unlimited)
float32[<=1] max_linear_kick_speed
# Max chip kick speed [m/s] (unset = unlimited)
float32[<=1] max_chip_kick_speed
# Distance from robot center to dribbler [m] (implicitly defines the opening angle and dribbler width)
float32[<=1] center_to_dribbler
# Movement limits
RobotLimits[<=1] limits
# Wheel angle configuration
RobotWheelAngles[<=1] wheel_angles
# Custom robot spec for specific simulators (the protobuf files are managed by the simulators)
# repeated google.protobuf.Any custom
