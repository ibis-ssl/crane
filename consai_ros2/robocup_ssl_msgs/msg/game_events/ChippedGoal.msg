# This message represents a ChippedGoal event in the game

# The team that tried to shoot the goal
uint8 by_team
# The bot that kicked the ball
uint32 by_bot
# The location where the ball entered the goal [m]
Vector2 location
# The location where the ball was kicked [m]
Vector2 kick_location
# The maximum height [m] of the ball, before it entered the goal and since the last kick [m]
float32 max_ball_height
