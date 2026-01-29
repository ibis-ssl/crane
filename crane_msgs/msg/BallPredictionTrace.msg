# ボール予測の予実管理データ

# 基準タイムスタンプ（ナノ秒）
int64 reference_timestamp_ns

# トレースID（一連の予測を追跡するためのID）
uint32 trace_id

# 予測点の履歴（各コンポーネントで追記）
crane_msgs/BallPredictionPoint[] prediction_points

# 過去の予測と実績の比較（リングバッファ、最大10件）
crane_msgs/BallPredictionActual[] actuals
