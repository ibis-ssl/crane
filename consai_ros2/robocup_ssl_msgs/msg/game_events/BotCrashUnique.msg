# The team that caused the crash
uint32 by_team
# The bot that caused the crash
uint32 violator
# The bot of the opposite team that was involved in the crash
uint32 victim
# The location of the crash (center between both bots) [m]
Vector2 location
# The calculated crash speed vector [m/s] of the two bots
float32 crash_speed
# The difference [m/s] of the velocity of the two bots
float32 speed_diff
# The angle [rad] in the range [0, π] of the bot velocity vectors
# an angle of 0 rad (  0°) means, the bots barely touched each other
# an angle of π rad (180°) means, the bots crashed frontal into each other
float32 crash_angle
