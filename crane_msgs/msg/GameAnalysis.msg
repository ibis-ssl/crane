# ゲーム分析情報
Kick[<=1] ongoing_kick

Slack[] our_slack
Slack[] their_slack

float32 ball_horizon

# sorted by score
FloatWithID[] pass_scores

# -1 means "not selected"
int32 pass_target_id

# ボール脅威（最優先で対応すべき脅威）
ThreatInfo ball_threat

# ロボット脅威（優先度順にソート）
ThreatInfo[] robot_threats

# 脅威割り当て結果
ThreatAssignment[] threat_assignments

# パス妨害推奨（存在する場合）
bool has_pass_disruption_target
uint8 pass_disruption_target_robot_id
geometry_msgs/Point pass_disruption_position
uint8 pass_disruption_defender_id

# 推奨守備者数
uint8 recommended_num_defenders

# 推奨役割（新規フィールド）
int32 recommended_attacker_id        # 推奨アタッカーID（-1は未選択）
float32 attacker_suitability_score   # アタッカー適性スコア
int32 recommended_pass_receiver_id   # 推奨パスレシーバーID（-1は未選択）

# SubAttacker推奨位置
geometry_msgs/Point recommended_sub_attacker_position
float32 sub_attacker_position_score
bool has_sub_attacker_position  # 有効な位置が存在するか

# パス計画（M2: シャドー運用・消費者なし。M3 以降で出し手/受け手が消費）
PassPlan pass_plan
