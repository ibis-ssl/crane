# A team has too many robots on the field

# The team that has too many robots
uint32 by_team

# Number of robots allowed at the moment
int32[<=1] num_robots_allowed  # optional

# Number of robots currently on the field
int32[<=1] num_robots_on_field  # optional

# The location of the ball at the moment when this foul occurred [m]
Vector2[<=1] ball_location  # optional
