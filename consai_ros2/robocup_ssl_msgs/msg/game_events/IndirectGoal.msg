# The team that tried to shoot the goal
uint32 by_team
# The bot that kicked the ball - at least the team must be set
uint32[<=1] by_bot  # optional
# The location where the ball entered the goal [m]
Vector2[<=1] location  # optional
# The location where the ball was kicked [m]
Vector2[<=1] kick_location  # optional
