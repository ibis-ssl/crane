# This message represents a Goal event in the game

# The team that scored the goal
uint8 by_team
# The team that shot the goal (different from by_team for own goals)
uint32 kicking_team
# The bot that shot the goal
uint32 kicking_bot
# The location where the ball entered the goal [m]
Vector2 location
# The location where the ball was kicked (for deciding if this was a valid goal) [m]
Vector2 kick_location
# The maximum height the ball reached during the goal kick (for deciding if this was a valid goal) [m]
float32 max_ball_height
# Number of robots of scoring team when the ball entered the goal (for deciding if this was a valid goal)
uint32 num_robots_by_team
# The UNIX timestamp [μs] when the scoring team last touched the ball
uint64 last_touch_by_team
# An additional message with e.g. a reason for invalid goals
string message
