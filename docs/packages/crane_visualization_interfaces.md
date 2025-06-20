# crane_visualization_interfaces

## 概要

Craneシステムの**可視化インターフェース定義**を提供するパッケージです。システム内部状態の可視化に必要なメッセージ定義とProtobuf変換機能を提供し、デバッグ・分析・観戦用の可視化システムを支援します。

## 主要機能

- **可視化メッセージ定義**: 可視化専用のメッセージ型定義
- **Protobuf変換**: 高効率なデータ転送形式への変換
- **統合可視化**: 複数データソースの統合表示
- **リアルタイム可視化**: 低遅延での可視化データ配信

## メッセージ定義

- **ObjectsArray.proto**: 可視化オブジェクト配列の定義
- **変換関数**: ROS 2メッセージ ⇔ Protobuf変換

## アーキテクチャ上の役割

Craneシステムの**可視化基盤**として、内部状態を外部可視化システムに効率的に伝達する役割を担います。

## 使用方法

```cpp
#include "crane_visualization_interfaces/manual_conversions.hpp"

// 可視化データの作成・変換
auto viz_data = createVisualizationData();
auto proto_data = convertToProto(viz_data);
```

## 最近の開発状況

🟢 **安定**: 可視化インターフェースとして成熟しており、新しい可視化データ型の追加や変換効率の向上が継続的に行われています。

---

**関連パッケージ**: [crane_visualization_aggregator](./crane_visualization_aggregator.md) | [consai_visualizer](./consai_visualizer.md)
