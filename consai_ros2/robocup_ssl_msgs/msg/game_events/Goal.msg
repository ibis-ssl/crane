# The team that scored the goal
uint32 by_team
# The team that shot the goal (different from by_team for own goals)
uint32[<=1] kicking_team  # optional
# The bot that shot the goal
uint32[<=1] kicking_bot  # optional
# The location where the ball entered the goal [m]
Vector2[<=1] location  # optional
# The location where the ball was kicked (for deciding if this was a valid goal) [m]
Vector2[<=1] kick_location  # optional
# The maximum height the ball reached during the goal kick (for deciding if this was a valid goal) [m]
float32[<=1] max_ball_height  # optional
# Number of robots of scoring team when the ball entered the goal (for deciding if this was a valid goal)
uint32[<=1] num_robots_by_team  # optional
# The UNIX timestamp [μs] when the scoring team last touched the ball
uint64[<=1] last_touch_by_team  # optional
# An additional message with e.g. a reason for invalid goals
string[<=1] message  # optional
