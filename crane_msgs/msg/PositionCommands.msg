# PositionCommands.msg
# session_planner → local_planner 用の位置指令配列メッセージ

std_msgs/Header header
bool on_positive_half
bool is_yellow
PositionCommand[] robot_commands

# 遅延監視用チェックポイント情報（PositionCommands全体レベル）
crane_msgs/DelayCheckpoints delay_checkpoints
