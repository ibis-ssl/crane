# Skill の設計契約

Skill は、1 台のロボットに対する行動を、セッションから実行できる単位にします。API の一覧や使用例は [`crane_robot_skills`](https://github.com/ibis-ssl/crane/tree/develop/crane_robot_skills) の実装を参照してください。

実装時に守ること:

- `SkillInterface::run()` は各周期に `Status::RUNNING`、`SUCCESS`、`FAILURE` のいずれかを返す。
- 単一動作は `SkillBase::update()`、状態遷移を持つ動作は `SkillBaseWithState` を使う。
- 入力は `WorldModelWrapper`、出力は `commander()` のロボット指令と `visualizer` の描画。`run()` の前後処理（指令初期化、キック無効化、描画 flush）を迂回しない。
- 位置はフィールド座標（m）、角度はラジアン（rad）。単位と軸は [座標系仕様](./coordinates.md) を正本とする。
- スキルを合成するときは、親の指令・可視化レイヤー・状態を壊さない。安全制約（停止、ドリブル距離、ペナルティエリア）は下位プランナと基底クラスに任せる。

主な参照先:

- [基底クラス](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/include/crane_robot_skills/skill_base.hpp)
- [スキル実装](https://github.com/ibis-ssl/crane/tree/develop/crane_robot_skills/src)
- [Attacker](./attacker.md) / [可視化](./visualizer.md)
