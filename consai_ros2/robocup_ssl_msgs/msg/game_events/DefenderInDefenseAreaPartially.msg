# This message represents a DefenderInDefenseAreaPartially event in the game

# The team that found guilty
uint8 by_team
# The bot that is partially inside the penalty area
uint32 by_bot
# The location of the bot
Vector2 location
# The distance [m] that the bot is inside the penalty area
float32 distance
# The location of the ball at the moment when this foul occurred [m]
Vector2 ball_location
