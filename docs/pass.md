# パス連携

通常プレーでは `PassPlan` を出し手・受け手の共通契約として使います。受け手の現在位置ではなく、計画された受領点へ直進パスを送り、受け手はキック前からその地点へ先回りします。

## 処理の流れ

1. `PassPlanMetric` が各受け手の現在点・周辺点を交互に評価します。受け手の先着可否と敵の迎撃を同じキック初速・ボール減速で判定し、最低スコアを満たす計画を選びます。
2. 割当処理が出し手・受け手を一組として確保します。固定割当などで実現できない場合は、その周期のパス実行を拒否します。
3. `Attacker` はシュートを優先し、パスを選ぶ場合は計画の受領点・初速をそのまま使います。`PassReceiverSession` は受領点へ移動し、キック後は実測ボール軌道に対して `Receive` を実行します。
4. 飛行中は計画の受け手・受領点を保持します。停止・方向逸脱・相手のキック・タイムアウトなどで解除します。フリーキックとGKの排出は専用の判断経路です。

## メッセージ契約

契約は [`PassPlan.msg`](https://github.com/ibis-ssl/crane/blob/develop/crane_msgs/msg/PassPlan.msg) が正本です。`pass_target_id` / `pass_scores` は比較用の旧評価として残しています。有効な計画がない場合、受け手セッションは従来の推薦を使いますが、出し手は旧評価でパスを代行しません。

迎撃評価は経路の離散サンプルによる近似です。計画の成立は実機での成功保証ではありません。

## 検証

「誰かに渡った」ではなく「予定した受け手が・予定した地点で受け取った」を確かめます。シナリオテスト `PASS_PLAN_AS_PLANNED` がキック時点の計画をラッチし、実際に蹴ったロボット・最初に触れた味方・接触点を計画と照合します。許容誤差はテスト側の定数です。

計画は `/world_model` に埋め込まれて配信されるだけで、採否の理由はどこにも出ません。テストは併走する記録プロセスで `/world_model` と `/robot_select_results` を購読し、共通ゲートの各条件と割当結果を時系列で残します。落ちた段はこの記録から特定します。

このテストと既存のパステストはシミュレータのばらつきが大きいため、CI マトリクスには入れていません。ローカルで複数試行し、成功率で判断してください。1 回の結果で可否を決めないでください。

実測で分かっている制約:

- 支配的な失敗要因は受領点の誤差ではなく、キック時点で計画が生き残っているかです。スコアは 10Hz で大きく変動するため、生成側は保持中に閾値を緩めるヒステリシスを持ちます。
- `own_goal_penalty` は遮蔽を考慮しない純幾何のため、フィールドのほぼ全域で上限に張り付きます。結果としてゴール角ボーナスが無い受領点は成立しません。
- キック初速を上げると敵の迎撃余地は減りますが、受け手が受領点へ間に合わなくなり候補がほとんど棄却されます。この二律背反は調整で消えません。

## 実装リファレンス

- [PassPlanMetric](https://github.com/ibis-ssl/crane/blob/develop/crane_game_analyzer/src/metrics/pass_plan_metrics.cpp)
- [PassReceiverSession](https://github.com/ibis-ssl/crane/blob/develop/crane_sessions/include/crane_sessions/pass_receiver_session.hpp)
- [Attacker](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/src/attacker.cpp)
- [オフェンス戦術](./offense.md) / [Attacker スキル](./attacker.md)
