# 脅威への守備者割り当て
# Sumatraの DefensePassDisruptionAssignment を参考に設計

# 割り当て対象の脅威
ThreatInfo threat

# 割り当てられた守備ロボットID配列
uint8[] defender_ids

# 守備戦略
uint8 STRATEGY_CENTER_BACK = 0       # センターバック（ゾーン守備）
uint8 STRATEGY_MAN_TO_MAN = 1        # マンツーマン
uint8 STRATEGY_PASS_DISRUPTION = 2   # パス妨害
uint8 defense_strategy

# 守備位置（defender_idsに対応）
geometry_msgs/Point[] protection_positions
