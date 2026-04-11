# 半面練習モード設定
# ros2 param set でランタイムにトグル可能

# 練習モード有効/無効
bool enabled false

# true: 正側(+X)でプレイ, false: 負側(-X)でプレイ
bool positive_side true

# true: 全キックを禁止（ボールが相手コートに行くのを防止）
bool kick_prohibition false

# true: FW/アタッカーの攻撃方向を反転（自陣ゴール方向に攻める）
# DF/ゴーリーには影響しない
bool reverse_attack false

# kick_prohibition=true 時でもキックを許可するスキル名リスト
# 例: ["goalie_skill"] → ゴーリーのみキック許可
string[] kick_allowed_skills

# true: STOP中でも通常速度で動作する（STOP速度制限を無視）
bool use_normal_speed true
