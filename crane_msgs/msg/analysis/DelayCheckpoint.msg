# 遅延監視システム用チェックポイント情報
# RobotCommandのstate_factorsパターンを参考に設計

# チェックポイント名（例: "vision_received", "ekf_updated", "world_model_published"）
string name

# 基準タイムスタンプからの相対時間（マイクロ秒単位）
# DelayCheckpoints.reference_timestamp_nsからの差分を記録
int32 relative_time_us

# オプション：追加の文字列情報（例: "30Hz", "robot_id:3"）
string value
