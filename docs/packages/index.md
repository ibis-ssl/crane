# パッケージ一覧

| パッケージ名 | 役割 |
|---|---|
| [crane_bringup](https://github.com/ibis-ssl/crane/tree/develop/crane_bringup) | システム統合起動とパラメータ・ノード管理 |
| [crane_game_analyzer](https://github.com/ibis-ssl/crane/tree/develop/crane_game_analyzer) | 試合状況のリアルタイム分析・定量評価・ヒステリシス安定化 |
| [crane_latency_estimator](https://github.com/ibis-ssl/crane/tree/develop/crane_latency_estimator) | システム内処理遅延・通信レイテンシの計測・推定 |
| [crane_local_planner](https://github.com/ibis-ssl/crane/tree/develop/crane_local_planner) | 局所経路計画と動的障害物・ルールエリア衝突回避 |
| [crane_msgs](https://github.com/ibis-ssl/crane/tree/develop/crane_msgs) | Craneシステム共通のROS 2メッセージ・サービス定義 |
| [crane_play_switcher](https://github.com/ibis-ssl/crane/tree/develop/crane_play_switcher) | 審判コマンド解釈とプレイ状況（PlaySituation）遷移判定 |
| [crane_robot_receiver](https://github.com/ibis-ssl/crane/tree/develop/crane_robot_receiver) | ロボットフィードバック受信・通信/バッテリー/ハードウェア健全性監視 |
| [crane_robot_skills](https://github.com/ibis-ssl/crane/tree/develop/crane_robot_skills) | 個別ロボット行動（Attacker, Goalie, Kick等）のスキル実装ライブラリ |
| [crane_sender](https://github.com/ibis-ssl/crane/tree/develop/crane_sender) | 実機ロボット（Ibis）および各種シミュレータへのコマンドパケット送信 |
| [crane_session_coordinator](https://github.com/ibis-ssl/crane/tree/develop/crane_session_coordinator) | ゲーム状態管理・設定駆動型ロボット役割（セッション）割り当て |
| [crane_sessions](https://github.com/ibis-ssl/crane/tree/develop/crane_sessions) | 攻守・セットプレー等のマルチロボット協調戦略（セッション）実装 |
| [crane_speaker](https://github.com/ibis-ssl/crane/tree/develop/crane_speaker) | 試合状況の音声アナウンス |
| [crane_teleop](https://github.com/ibis-ssl/crane/tree/develop/crane_teleop) | ジョイスティック・キーボードによるロボット手動遠隔操作 |
| [crane_visualization_interfaces](https://github.com/ibis-ssl/crane/tree/develop/crane_visualization_interfaces) | SVG可視化メッセージ定義・描画ラッパー・データ集約とスナップショット配信 |
| [crane_world_model_publisher](https://github.com/ibis-ssl/crane/tree/develop/crane_world_model_publisher) | Vision/Trackerデータ統合・3Dボール物理・世界モデル配信 |
| [crane_comm](https://github.com/ibis-ssl/crane/tree/develop/utility/crane_comm) | UDP/マルチキャスト通信・診断付きパブリッシャー等の通信基盤 |
| [crane_geometry](https://github.com/ibis-ssl/crane/tree/develop/utility/crane_geometry) | 幾何計算・図形交差・座標変換（Eigen/Boost.Geometry統合） |
| [crane_lint_common](https://github.com/ibis-ssl/crane/tree/develop/utility/crane_lint_common) | C++/Python/ROS 2コーディング規約および静的解析共通設定 |
| [crane_msg_wrappers](https://github.com/ibis-ssl/crane/tree/develop/utility/crane_msg_wrappers) | WorldModelWrapper等のメッセージ操作・変換ラッパー |
| [crane_physics](https://github.com/ibis-ssl/crane/tree/develop/utility/crane_physics) | ボール物理シミュレーション・ロボット運動学・台形速度プロファイル |
| [crane_utils](https://github.com/ibis-ssl/crane/tree/develop/utility/crane_utils) | 時間計測・パラメータ操作・パッケージパス解決等の共通ユーティリティ |
| [robocup_ssl_comm](https://github.com/ibis-ssl/crane/tree/develop/consai_ros2/robocup_ssl_comm) | SSL公式通信プロトコル（Vision, Referee）の送受信 |
| [robocup_ssl_msgs](https://github.com/ibis-ssl/crane/tree/develop/consai_ros2/robocup_ssl_msgs) | SSL公式プロトコル（Protobuf）から生成されたROS 2メッセージ |
| [closest_point_vendor](https://github.com/ibis-ssl/crane/tree/develop/3rdparty/closest_point_vendor) | 最近点幾何計算ライブラリのベンダーパッケージ |
| [matplotlib_cpp_17_vendor](https://github.com/ibis-ssl/crane/tree/develop/3rdparty/matplotlib_cpp_17_vendor) | C++17対応matplotlib描画ライブラリのベンダーパッケージ |
| [rvo2_vendor](https://github.com/ibis-ssl/crane/tree/develop/3rdparty/rvo2_vendor) | RVO2分散的衝突回避アルゴリズムのベンダーパッケージ |
| [crane_bag](https://github.com/ibis-ssl/crane/tree/develop/crane_bag) | C++ rosbag2解析CLIツール |
| [crane_mcap_tools](https://github.com/ibis-ssl/crane/tree/develop/crane_mcap_tools) | Pythonベースのrosbag/MCAP解析・SVG動画生成ツール |
| [crane_web_debugger](https://github.com/ibis-ssl/crane/tree/develop/crane_web_debugger) | WebSocketデバッグブリッジおよびWebフロントエンドUI（Viewer・Annotation・Robot Manager を同一オリジン 8090 で配信） |
