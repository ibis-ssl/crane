# 脅威情報
# Sumatraの DefenseThreat を参考に設計

# 脅威タイプ
uint8 THREAT_TYPE_BALL = 0
uint8 THREAT_TYPE_ROBOT = 1
uint8 threat_type

# 脅威元情報
uint8 source_robot_id              # THREAT_TYPE_ROBOTの場合のロボットID
geometry_msgs/Point source_position

# 脅威ライン（脅威元からゴールへの線分）
geometry_msgs/Point threat_line_start
geometry_msgs/Point threat_line_end

# 防御ライン（守備配置用、存在しない場合は has_protection_line=false）
geometry_msgs/Point protection_line_start
geometry_msgs/Point protection_line_end
bool has_protection_line

# 脅威速度ベクトル
geometry_msgs/Vector3 velocity

# 脅威評価スコア（0.0-1.0、高いほど危険）
float32 threat_rating

# 脅威評価の内訳（デバッグ/可視化用）
float32 score_redirect_angle     # リダイレクト角度スコア
float32 score_pen_area_border    # ペナルティエリア境界スコア
float32 score_facing_goal        # ゴール方向を向いている度合い
float32 score_ball_access        # ボールへのアクセスしやすさ

# ソースタイプ（ボール脅威の詳細分類）
uint8 SOURCE_TYPE_BALL = 0
uint8 SOURCE_TYPE_GOAL_SHOT = 1
uint8 SOURCE_TYPE_PASS_RECEIVE = 2
uint8 SOURCE_TYPE_BOT_CLOSE_TO_BALL = 3
uint8 source_type
