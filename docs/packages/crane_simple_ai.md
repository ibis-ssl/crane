# SimpleAI (SimpleAITactic)

## 概要

Craneシステムの**デバッグ・テスト用AIインターフェース**です。現在は `crane_tactics` パッケージ内の `SimpleAITactic` として実装されています。各ロボットに対して個別のスキルを直接指定して実行させる機能を提供し、主にGUIやデバッグツールからの個別動作テストに使用されます。

## 主要機能

- **スキル実行サーバー**: Action (`/simple_ai/skill_execution`) を通じた動的なスキル実行
- **全スキル対応**: `crane_robot_skills` に定義された主要なスキルをすべて実行可能
- **パラメータ対応**: Action経由での動的なスキルパラメータ変更
- **デバッグ支援**: Web GUI等からのロボット個別操作

## アーキテクチャ上の役割

メインの戦術決定システムを通さず、外部から特定のロボットに特定のスキルを実行させるための**バイパスインターフェース**として機能します。

## 使用方法

通常、`crane_tactic_coordinator` の設定で `simple_ai` タクティックを有効にすることで利用可能になります。

```yaml
# unified_session_config.yaml 例
situations:
  DEBUG:
    sessions:
      - name: simple_ai
        capacity: 11
```

### Actionを通じた実行

`crane_msgs/action/SkillExecution` メッセージを使用して、実行したいスキル名と対象ロボットIDを指定します。

## 最近の開発状況

🟢 **安定**: `crane_planner_plugins` から `crane_tactics` への移行に伴い、タクティックの一つとして統合されました。Webベースのデバッグツールとの連携が強化されています。

---

**関連パッケージ**: [crane_tactics](./crane_tactics.md) | [crane_robot_skills](./crane_robot_skills.md) | [crane_debug_tools](./crane_debug_tools.md)

---

**関連パッケージ**: [crane_tactic_coordinator](./crane_tactic_coordinator.md) | [crane_tactics](./crane_tactics.md)
