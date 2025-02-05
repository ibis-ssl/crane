# Feedback from a robot

# Id of the robot
uint32 id
# Has the dribbler contact to the ball right now
bool[<=1] dribbler_ball_contact
# Custom robot feedback for specific simulators (the protobuf files are managed by the simulators)
# optional google.protobuf.Any custom = 3;
