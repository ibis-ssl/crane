#
# crane_msgs/msg/RoleCommand.msg
#
# for use in : RoleCommands.msg
#

uint8 role_id

crane_msgs/SubRole[] sub_role
string[] param_names
float32[] params
