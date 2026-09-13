# パス連携

パス先の選定と受け手の準備は別の指標を使います。両者を同じ ID として扱わないことが、この連携の重要な契約です。

## 処理の流れ

1. `PassTargetMetric` が利用可能な受け手の `pass_scores` を計算し、ボールの停止位置または予測停止位置を起点にヒステリシス付きで `pass_target_id` を選びます。
2. `Attacker` は `pass_target_id` とスコアを確認し、障害物に応じてストレートキックまたはチップキックを実行します。未選択・低スコアならパスしません。
3. `PassReceiverSession` は `recommended_pass_receiver_id` を優先して受け手を予約します。キック前は停止してボールを見つつ、ボール移動または味方のキック検知で `Receive` を実行します。

## メッセージ契約

[`GameAnalysis.msg`](https://github.com/ibis-ssl/crane/blob/develop/crane_msgs/msg/GameAnalysis.msg) の主なフィールド:

- `pass_scores`: パス候補のスコア（降順）。
- `pass_target_id`: `Attacker` が使う選定先（未選定は `-1`）。
- `recommended_pass_receiver_id`: 受け手セッションへの推薦（未選定は `-1`）。

## 実装リファレンス

- [PassTargetMetric](https://github.com/ibis-ssl/crane/blob/develop/crane_game_analyzer/src/metrics/pass_target_metrics.cpp)
- [PassReceiverSession](https://github.com/ibis-ssl/crane/blob/develop/crane_sessions/include/crane_sessions/pass_receiver_session.hpp)
- [Attacker](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/src/attacker.cpp)
- [オフェンス戦術](./offense.md) / [Attacker スキル](./attacker.md)
