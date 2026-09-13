# オフェンス戦術システム

## 目的

相手陣形を崩して得点を狙うため、ボール保持ロボット（Attacker）と支援ロボット（SubAttacker / Forward）を協調動作させます。

## 戦術構成と連携規約

- **Attacker（メイン攻撃）**:
  - ボールを保持し、ゴールが見えればシュート、塞がれていればパスを選択。
  - 詳細は [Attackerスキル](./attacker.md) を参照。
- **SubAttacker / Forward（支援・ポジショニング）**:
  - アタッカーがボールを保持している間、パスコースを確保しつつこぼれ球を拾える位置へ動的にポジショニング。
- **Kick / Receive（連携実行）**:
  - パスライン上の障害物有無に応じてストレートキックとチップキックを自動選択。
  - パス連携の安定化契約については [パス連携](./pass.md) を参照。

## 安全・ルール制約規約

- **オーバードリブル防止**: `SkillBaseWithState` がボール保持中の移動距離を積算し、0.5 m を超えると `stopHere()` を発行する。詳細は [ルール制約](./rule.md) を参照。
- **相手ペナルティエリア回避**: シュート・パス時であっても相手ペナルティエリア内への進入・接触を回避。

## 実装リファレンス

- アタッカースキル: [attacker.hpp](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/include/crane_robot_skills/attacker.hpp)
- サブアタッカースキル: [sub_attacker.hpp](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/include/crane_robot_skills/sub_attacker.hpp)
- 関連ドキュメント: [Attacker スキル](./attacker.md) | [パス連携](./pass.md) | [ルール制約](./rule.md)
