# HumanAnnotation.msg
# 人間による試合/デバッグ中のリアルタイムアノテーション

std_msgs/Header header

# カテゴリ定数
uint8 CATEGORY_ISSUE = 0        # 問題/バグの発見
uint8 CATEGORY_OBSERVATION = 1  # 観察・気づき
uint8 CATEGORY_QUESTION = 2     # 後で確認したい疑問
uint8 CATEGORY_POSITIVE = 3     # 良かった動き
uint8 CATEGORY_STRATEGY = 4     # 戦略的なメモ
uint8 CATEGORY_TIMING = 5       # タイミング関連
uint8 CATEGORY_CUSTOM = 10      # カスタムカテゴリ

# 重要度定数
uint8 PRIORITY_LOW = 0
uint8 PRIORITY_MEDIUM = 1
uint8 PRIORITY_HIGH = 2
uint8 PRIORITY_CRITICAL = 3

# 基本フィールド
uint8 category
uint8 priority
string label                    # 短いラベル（ボタンテキスト）
string description              # 詳細説明（音声入力や手入力）

# 時刻関連（リワインド機能対応）
int64 event_timestamp_ns        # イベント発生時刻（ROSタイム、ナノ秒）
int64 client_timestamp_ms       # クライアント側タイムスタンプ
int64 time_offset_ms            # サーバーとクライアントの時刻差

# コンテキスト情報（オプション）
bool has_position
geometry_msgs/Point position    # フィールド上の関連位置

bool has_robot_context
uint8[] related_robot_ids       # 関連ロボットID
bool[] robot_is_ours            # 自チームかどうか

# 追加メタデータ（JSON形式、拡張用）
string metadata_json
