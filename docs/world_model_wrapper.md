# WorldModelWrapper

## 目的

`WorldModelWrapper` は、ROS 2トピック（`/world_model`）を自動購読し、ロボット・ボール（3D物理モデル統合済み）・フィールド幾何・試合状況（PlaySituation）への型安全な統合アクセスを提供するクラスです。

## 初期化と利用手順

ROS 2ノード内で初期化し、更新コールバックまたはポーリングによって最新データを取得します。

```cpp
#include <crane_msg_wrappers/world_model_wrapper.hpp>

// ノード初期化時にインスタンス化（自動的にサブスクライバが生成される）
world_model = std::make_shared<crane::WorldModelWrapper>(*this);

// 更新コールバックの登録
world_model->addCallback([this]() {
  const auto & ball = world_model->ball();
  auto available_robots = world_model->ours().robotsWhere().available().get();
});
```

## 規約と設計原則

- **座標系・単位規約**: 全てフィールド座標系（原点は中央、長さはメートル [m]、角度はラジアン [rad]、反時計回りが正）。詳細は [座標系仕様](./coordinates.md) を参照。
- **ロボット可用性契約**:
  - `available()`: 基本判定（`(available_vision || available_tracker) && available_hardware`）
  - `availableStrict()`: 厳密判定（`available_vision && available_hardware && available_feedback`）
  - `availableLoose()`: 緩和判定（`available_vision || available_tracker`）
  - 詳細は [診断システム（Diagnostics）](./diagnostics.md) を参照。
- **更新順序**: `addCallback()` のコールバックは世界モデル更新の直後に実行される。別 executor から同じインスタンスへアクセスする場合は、呼び出し側で同期すること。

## 実装リファレンス

- クラス宣言: [world_model_wrapper.hpp](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_msg_wrappers/include/crane_msg_wrappers/world_model_wrapper.hpp)
- 実装: [world_model_wrapper.cpp](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_msg_wrappers/src/world_model_wrapper.cpp)
- ロボットクエリ: [robots_query.hpp](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_msg_wrappers/include/crane_msg_wrappers/robots_query.hpp)
- ボール物理統合: [ボールトラッキングシステム設計書](./ball_tracking_system.md)
