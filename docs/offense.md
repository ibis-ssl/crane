# オフェンス戦術システム

> **最終更新**: 2026年1月
> **関連パッケージ**: [crane_robot_skills](./packages/crane_robot_skills.md), [crane_tactics](./packages/crane_tactics.md)

Craneシステムにおけるオフェンス戦術は、`crane_robot_skills`パッケージのスキルベースアーキテクチャと`crane_tactics`の戦略Tacticにより実現されています。

## アーキテクチャ概要

### スキルベース設計

オフェンス行動は、個別のロボットスキルの組み合わせとして実装されます：

- **Attacker**: 複合的な攻撃行動（状態遷移ベース）
- **SubAttacker**: アタッカーを支援するポジショニングと攻撃行動
- **Kick**: ボールキック動作（ストレート・チップ対応。独立したスキルとして他から呼び出される）
- **Receive**: ボール受け取り専用スキル
- **Dribble**: （独立したスキルではなく、各スキル内で `command->dribble()` を呼び出すことで実現）

### 状態遷移システム

各スキルは `SkillBaseWithState` を継承した状態機械として実装され、ゲーム状況に応じて適切な行動を選択します。

## 主要オフェンススキル

### Attacker（複合攻撃行動）

**実装場所**: `crane_robot_skills/include/crane_robot_skills/attacker.hpp`

- **機能**:
  - `PassTargetSelector` と連携したパス実行
  - ゴールが見える場合は `GoalKick` を実行
  - ボールが自分に向かっている場合は `Receive` を実行
  - オーバードリブル（0.5m超）の自動監視と停止

### SubAttacker（支援攻撃行動）

**実装場所**: `crane_robot_skills/include/crane_robot_skills/sub_attacker.hpp`

- **機能**:
  - アタッカーがボールを保持している間の最適なポジショニング
  - こぼれ球の回収準備
  - パスコースの確保

### Kick（キック動作）

- **機能**:
  - ストレートキック・チップキックの切り替え
  - ドリブルを併用したキック（助走中のボール保持向上）
  - 障害物（敵ロボット）を考慮した自動チップキック選択（`configurePassKick` 経由）

## 戦略レベルの統合 (Tactic)

### crane_tactics との連携

**SimpleAITactic**:

- 最もシンプルな攻撃戦略実装
- ボールに最も近いロボットに `Attacker` スキルを割り当て

**FormationTactic**:

- 攻撃時のロボット配置（FW, MF等）を管理
- 状況に応じた動的なポジショニング

## 実装の詳細

各スキルの実装詳細は以下のドキュメントを参照してください：

- **[crane_robot_skills](./packages/crane_robot_skills.md)** - 個別スキルの実装仕様
- **[crane_tactics](./packages/crane_tactics.md)** - 戦略Tacticの統合実装
- **[attacker.md](./attacker.md)** - Attackerスキルの詳細な状態遷移
