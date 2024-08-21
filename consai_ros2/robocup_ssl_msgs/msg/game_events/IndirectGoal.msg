# This message represents an IndirectGoal event in the game

# The team that tried to shoot the goal
uint8 by_team
# The bot that kicked the ball - at least the team must be set
uint32 by_bot
# The location where the ball entered the goal [m]
Vector2 location
# The location where the ball was kicked [m]
Vector2 kick_location
