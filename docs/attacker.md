# Attacker スキル

`SkillBaseWithState` を基底に、ボールの受け取り、シュート、パスを 1 台のロボットで切り替えます。状態名・遷移条件の正本は [`attacker.hpp`](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/include/crane_robot_skills/attacker.hpp) と実装です。

## 状態と責務

- `ENTRY_POINT`: 内部スキルとパス先をリセットし、次の行動を選ぶ。
- `RECEIVE`: 移動中のボールを `Receive` に委譲して受け取る。ボールが停止・通過・接触したら `ENTRY_POINT` へ戻る。
- `KICK`: ゴール角度、`game_analysis.pass_plan`、ボール位置に応じて `GoalKick` または `KickOld` を実行する。パスでは計画された受領点・初速を使う。ボールが動いていてもロボットが近い間は状態を維持し、再遷移の発振を避ける。

## 安全制約

`SkillBaseWithState` がボール保持中の移動距離を積算し、`OVER_DRIBBLE_DISTANCE_THRESHOLD`（現在 0.5 m）を超えると `command->stopHere()` を発行します。ルール制約の責務分担は [ルール制約](./rule.md) を参照してください。

実装:

- [attacker.hpp](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/include/crane_robot_skills/attacker.hpp)
- [attacker.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/src/attacker.cpp)
- [パス連携](./pass.md) / [Skill の設計契約](./skill.md)
