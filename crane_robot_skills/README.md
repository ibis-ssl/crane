# crane_robot_skills

## 役割

個別ロボットの戦術的行動（Attacker, Goalie, Marker 等）を統一インターフェースで実装・提供するスキルライブラリです。

## 固有の制約

- 基本行動は `SkillBase`、状態機械を伴う複合行動は `SkillBaseWithState` を継承すること。
- コマンド生成は `commander()` 経由で行い、実行ステータス（`SUCCESS`, `FAILURE`, `RUNNING`）を返すこと。
- オーバードリブル監視（0.5m超での自動停止）などのルール制約を厳格に実装すること。

## トピックガイド

- [Skill の設計契約](../docs/skill.md)
- [Attackerスキル](../docs/attacker.md)

## リンク

- ソースディレクトリ: [crane_robot_skills](https://github.com/ibis-ssl/crane/tree/develop/crane_robot_skills)
