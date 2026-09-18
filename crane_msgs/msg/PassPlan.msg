# パス計画（誰が・誰に・どの受領点へ・どんなキックで）
# 分析層で一元計算し、通常プレーの出し手・受け手が同じ計画を消費する。

# --- 状態機械（KICK_READY / COMPLETED は予約） ---
uint8 STATE_INACTIVE=0        # 有効な計画なし
uint8 STATE_PLANNING=1        # キック前・計画を継続更新中
uint8 STATE_KICK_READY=2      # キック実行可（予約）
uint8 STATE_BALL_IN_FLIGHT=3  # ボール飛行中・受領点保持
uint8 STATE_COMPLETED=4       # パス完了（予約）
uint8 STATE_ABORTED=5         # 役割割当が計画と一致せず実行を拒否

uint32 plan_id                # 計画世代（再開または出し手・受け手変更時に増加）
uint8 state                   # STATE_* のいずれか

int32 kicker_id               # 出し手ロボットID（-1 は未定）
int32 receiver_id             # 受け手ロボットID（-1 は未定）
geometry_msgs/Point receive_point  # 受領点（受け手現在位置とは限らない＝走り込み先）

float32 kick_speed            # 計画キック初速 [m/s]
bool is_chip                  # チップキックか（現在の計画は false 固定）
float32 chip_distance         # チップ着地距離 [m]（is_chip 時のみ有効）

float32 ball_travel_time      # ボール到達所要時間 [s]（planStraightPass 由来）
float32 receiver_travel_time  # 受け手の受領点到達所要時間 [s]

float32 score                 # パス総合スコア（ratePassCandidate）
# スコア内訳（可視化・上位判断で参照。PassRating と対応）
float32 distance_factor
float32 goal_angle_bonus
float32 own_goal_penalty
float32 their_goal_factor
float32 intercept_score
float32 shadow_score
