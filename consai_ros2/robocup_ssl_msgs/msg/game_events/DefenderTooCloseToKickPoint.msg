# This message represents a DefenderTooCloseToKickPoint event in the game

# The team that was found guilty
uint8 by_team
# The bot that violates the distance to the kick point
uint32 by_bot
# The location of the bot [m]
Vector2 location
# The distance [m] from bot to the kick point (including the minimum radius)
float32 distance
