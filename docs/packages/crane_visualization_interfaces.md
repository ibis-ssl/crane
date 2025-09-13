# crane_visualization_interfaces

## 概要

Craneシステムの可視化向けROS 2メッセージを提供するパッケージです。従来のProtobufベース実装から、標準のROS 2 msgベース実装へ移行しました。

## 特徴

- 可視化専用のメッセージ定義（rosidl）
- `std_msgs/Header` による時系列管理に対応
- 依存はROS 2の標準メッセージのみに簡素化

## 提供メッセージ（スナップショット/更新）

- `SvgLayerSnapshot.msg`（スナップショット要素）
  - レイヤの完全表現
  - `layer`, `svg_primitives[]`
- `SvgSnapshot.msg`（スナップショット）
  - 複数レイヤの完全状態を配信
  - `header`, `epoch`, `seq`, `layers[]`
- `SvgLayerUpdate.msg`（更新要素）
  - レイヤー単位の増分更新
  - `layer`, `operation`（`replace` | `append` | `clear`）, `svg_primitives[]`
- `SvgUpdates.msg`（更新）
  - 複数レイヤ更新のまとめ配信
  - `header`, `epoch`, `seq`, `updates[]`

※ 参照型（例: `SvgLayerUpdate`）は同パッケージ内の `.msg` として定義してください。

## 使い方（購読/配信）

```bash
# ビルドと環境読み込み（ワークスペースルートで）
colcon build --packages-select crane_visualization_interfaces
source install/local_setup.bash

# トピック例（実際の統合側で定義）
ros2 topic echo /aggregated_svgs crane_visualization_interfaces/msg/SvgSnapshot
ros2 topic echo /visualizer_svgs crane_visualization_interfaces/msg/SvgUpdates
```

関連: [crane_visualization_aggregator](./crane_visualization_aggregator.md) | [consai_visualizer](./consai_visualizer.md)
