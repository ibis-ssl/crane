# 速度計画の予実管理データ（上流から下流まで共通）

# 基準タイムスタンプ（ナノ秒）
int64 reference_timestamp_ns

# トレースID（一連のコマンドを追跡するためのID）
uint32 trace_id

# 計画点の履歴（各レイヤーで追記）
crane_msgs/VelocityPlanPoint[] plan_points

# 速度修正の履歴（RVO2、senderの加速度制限など）
crane_msgs/VelocityCorrection[] corrections

# 過去の計画と実績の比較（リングバッファ、最大10件）
crane_msgs/VelocityPlanActual[] actuals
