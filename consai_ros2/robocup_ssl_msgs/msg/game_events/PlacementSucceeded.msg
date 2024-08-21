# The team that did the placement
uint8 by_team

# The time [s] taken for placing the ball
float32[<=1] time_taken  # optional

# The distance [m] between placement location and actual ball position
float32[<=1] precision  # optional

# The distance [m] between the initial ball location and the placement position
float32[<=1] distance  # optional
