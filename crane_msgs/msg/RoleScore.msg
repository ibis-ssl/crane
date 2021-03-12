#
# crane_msgs/RoleScore.msg
#
# for use in : RoleScores.msg
#

# role id
# 0 : Defender
# 1 : ...
uint8 DEFENDER=0
uint8 PASSER=1
uint8 PASSCUTTER=2  # meaning interceptor
uint8 BALLSTEALER=3
uint8 BALLPLACER=4
uint8 NOTBALLPLACER=5
uint8 IDLER=6

uint8 role_id

# PARAM_ID
# 0  : robot id
# 1  : x
# 2  : y
# 3  : ...

uint8 param_num

uint8[] param_id
uint32[] param_size
float32[] unit
float32[] offset 

float32[] score
