# ルール制約

Crane は、公式ルールの写しではなく、違反を避けるための実装上の責務をまとめます。公式ルールや実行時の閾値が変わったときは、まずソースコード・設定・テストを更新し、このページは参照先だけを保ちます。

| 対象 | 実装上の責務 | SSOT |
| --- | --- | --- |
| オーバードリブル | ボール保持中の移動距離を積算し、閾値を超えたら `stopHere()` を発行する（現在の閾値は 0.5 m）。 | [`SkillBaseWithState`](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/include/crane_robot_skills/skill_base.hpp)、[`Attacker`](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/src/attacker.cpp) |
| ペナルティエリア・プレイスメント・停止 | 世界モデルのルール状態と指令設定に応じ、局所プランナが進入禁止領域と速度制約を適用する。全体のマージンを広げる変更は、まずテストと実機影響を確認する。 | [`rvo2_planner.hpp`](https://github.com/ibis-ssl/crane/blob/develop/crane_local_planner/include/crane_local_planner/rvo2_planner.hpp)、[`rvo2_planner.cpp`](https://github.com/ibis-ssl/crane/blob/develop/crane_local_planner/src/rvo2_planner.cpp) |
| 衝突回避 | RVO2 と速度・加速度制約で動的な障害物を避ける。 | [局所経路計画](./rvo2_local_planner.md) |
| キック速度 | `KickerModel` と `kicker_physics.yaml` の設定から指令値を決める。 | [`kicker_physics.yaml`](https://github.com/ibis-ssl/crane/blob/develop/crane_world_model_publisher/config/kicker_physics.yaml)、[`KickerModel`](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_physics/include/crane_physics/kicker_model.hpp) |

ルール判定に関する変更では、シミュレーションだけでなく、停止・配置・キックを含むシナリオテストも実行します。実装の詳細は [局所経路計画](./rvo2_local_planner.md)、[Attacker スキル](./attacker.md)、[シナリオテスト](../scenario_test/README.md) を参照してください。
