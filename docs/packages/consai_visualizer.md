# consai_visualizer

## 概要

RoboCup Small Size League専用の**フィールド可視化GUIシステム**です。Python Qt baseのrqt統合プラグインとして、SSL試合の可視化、フィールド幾何の表示、ロボット・ボール状態のリアルタイム表示を提供します。

## 主要機能

- **SSL専用可視化**: フィールド・ロボット・ボール・軌道の表示
- **rqt統合**: ROS 2 rqtプラグインアーキテクチャ
- **リアルタイム表示**: 60Hzでの高頻度更新
- **インタラクティブ操作**: マウス操作による視点変更・拡大縮小

## GUI機能

- **フィールド表示**: SSL標準フィールドの正確な描画
- **ロボット可視化**: チーム色・ID・向き・状態の表示
- **ボール軌道**: 現在位置・予測軌道・物理状態
- **戦術表示**: フォーメーション・役割・移動計画

## アーキテクチャ上の役割

Craneシステムの**専用可視化インターフェース**として、SSL特化の表示機能を提供し、開発・デバッグ・試合観戦での視覚的理解を支援します。

## 使用方法

```bash
# rqt統合起動
rqt --standalone consai_visualizer

# 直接起動
ros2 run consai_visualizer consai_visualizer
```

## 技術スタック

- **Python Qt**: QtWidgets、QPainter
- **rqt**: ROS 2 GUI統合フレームワーク
- **ROS 2**: rclpy、python_qt_binding

## 最近の開発状況

🟢 **安定**: SSL可視化システムとして成熟しており、新しい表示機能の追加やユーザビリティの向上が継続的に行われています。

---

**関連パッケージ**: [crane_visualization_interfaces](./crane_visualization_interfaces.md) | [crane_visualization_aggregator](./crane_visualization_aggregator.md)
