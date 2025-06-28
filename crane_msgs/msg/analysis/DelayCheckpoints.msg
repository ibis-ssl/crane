# 遅延監視システム用チェックポイント集合
# 基準タイムスタンプと相対時間での遅延チェックポイント管理

# 基準タイムスタンプ（ナノ秒単位、std::chrono::steady_clock::now()で取得）
# 最初のチェックポイント追加時に設定される絶対時刻
int64 reference_timestamp_ns

# チェックポイント配列
# 各チェックポイントは基準からの相対時間（マイクロ秒）で記録
DelayCheckpoint[] checkpoints
