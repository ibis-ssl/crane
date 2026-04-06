# キック予測の予実管理データ

int64 reference_timestamp_ns
uint32 trace_id

# 予測点（キック発生時に記録）
crane_msgs/KickPredictionPoint[<=1] prediction_point

# 実績（ボール停止時に記録）
crane_msgs/KickPredictionActual[<=1] actual
