# パス連携

通常プレーでは `PassPlan` を出し手・受け手の共通契約として使います。受け手の現在位置ではなく、計画された受領点へ直進パスを送り、受け手はキック前からその地点へ先回りします。

## 処理の流れ

1. `PassPlanMetric` が各受け手の現在点・周辺点を交互に評価します。受け手の先着可否と敵の迎撃を同じキック初速・ボール減速で判定し、最低スコアを満たす計画を選びます。
2. 割当処理が出し手・受け手を一組として確保します。固定割当などで実現できない場合は、その周期のパス実行を拒否します。
3. `Attacker` はシュートを優先し、パスを選ぶ場合は計画の受領点・初速をそのまま使います。`PassReceiverSession` は受領点へ移動し、キック後は実測ボール軌道に対して `Receive` を実行します。
4. 飛行中は計画の受け手・受領点を保持します。停止・方向逸脱・相手のキック・タイムアウトなどで解除します。フリーキックとGKの排出は専用の判断経路です。

## メッセージ契約

契約は [`PassPlan.msg`](https://github.com/ibis-ssl/crane/blob/develop/crane_msgs/msg/PassPlan.msg) が正本です。`pass_target_id` / `pass_scores` は比較用の旧評価として残しています。有効な計画がない場合、受け手セッションは従来の推薦を使いますが、出し手は旧評価でパスを代行しません。

迎撃評価は経路の離散サンプルによる近似です。計画の成立は実機での成功保証ではないため、検証時は可視化の受領点・迎撃スコアと実際のボール軌道を照合してください。

## 実装リファレンス

- [PassPlanMetric](https://github.com/ibis-ssl/crane/blob/develop/crane_game_analyzer/src/metrics/pass_plan_metrics.cpp)
- [PassReceiverSession](https://github.com/ibis-ssl/crane/blob/develop/crane_sessions/include/crane_sessions/pass_receiver_session.hpp)
- [Attacker](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/src/attacker.cpp)
- [オフェンス戦術](./offense.md) / [Attacker スキル](./attacker.md)
