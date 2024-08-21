# This message represents a BotPushedBot event in the game

# The team that pushed the other team
uint8 by_team
# The bot that pushed the other bot
uint32 violator
# The bot of the opposite team that was pushed
uint32 victim
# The location of the push (center between both bots) [m]
Vector2 location
# The pushed distance [m]
float32 pushed_distance
