# VelocityCommands.msg
# local_planner → sender 用の速度指令配列メッセージ

std_msgs/Header header
bool on_positive_half
bool is_yellow
VelocityCommand[] robot_commands

# 遅延監視用チェックポイント情報（VelocityCommands全体レベル）
crane_msgs/DelayCheckpoints delay_checkpoints
