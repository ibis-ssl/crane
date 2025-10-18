# crane_simple_ai

## 概要

Craneシステムの**簡易AI制御システム**として、基本的なAI機能とCraneCommanderを提供するパッケージです。開発・デバッグ用途やシンプルな制御が必要な場面で使用されます。

## 主要機能

- **CraneCommander**: 基本的なロボット指揮機能
- **簡易AI**: 最小限のAI制御ロジック
- **デバッグ支援**: 開発時のテスト・デバッグ機能
- **フォールバック**: メインシステム不調時の予備システム

## コンポーネント

- **CraneCommander**: シンプルなロボット制御
- **SimpleAINode**: 基本AI制御ノード
- **Qt UI**: 操作用簡易インターフェース

## アーキテクチャ上の役割

メインのplanner_plugins/session_controllerシステムの**代替・補助システム**として、シンプルな制御やテスト環境での動作を提供します。

## 使用方法

```bash
# 簡易AI起動
ros2 run crane_simple_ai simple_ai_node
```

## 最近の開発状況

🟢 **安定**: 補助システムとして最小限の機能を維持。主要開発はメインAIシステムに集中しているため、保守的な更新が中心。

---

**関連パッケージ**: [crane_session_controller](./crane_session_controller.md) | [crane_planner_plugins](./crane_planner_plugins.md)
