# modern_orca

## 概要

`modern_orca`は、拡張可能な制約システムとプラグインサポートを備えた、最適往復衝突回避（ORCA: Optimal Reciprocal Collision Avoidance）アルゴリズムのモダンなC++20実装です。マルチエージェント環境におけるリアルタイムな衝突回避を実現します。

## 主要機能

### モダンC++20設計

- スマートポインタ、コンセプト、レンジによる型安全設計
- ヘッダーオンリーのテンプレートベース実装
- ゼロコスト抽象化とインライン化による最適化

### 拡張可能アーキテクチャ

- プラグインシステムによるカスタム制約の追加
- 半平面制約APIによる速度制約の直接制御
- 派生メソッドとソルバーのカスタマイズ

### 高性能

- OpenMPによるマルチスレッド並列処理
- SIMD対応のベクトル化演算
- カスタムアロケータとオブジェクトプーリング

## アーキテクチャ上の役割

**依存レイヤ**: 3rdpartyベンダー（Layer 0）

- RVO2の後継として、より柔軟で拡張性の高い衝突回避ライブラリ
- `crane_local_planner`での活用を想定
- `crane_geometry`と統合して使用

## コアコンポーネント

### エージェント (`agents/`)

- **CircularAgent**: 円形の衝突形状を持つエージェント
- **PolygonAgent**: 任意多角形の衝突形状を持つエージェント
- テンプレートベースの拡張可能な設計

### 制約 (`constraints/`)

- **ORCAConstraint**: 標準的なORCA制約
- **HalfPlaneConstraint**: カスタム半平面制約
- プラグインによる独自制約の追加

### シミュレータ (`simulator.hpp`)

- メインシミュレーションオーケストレータ
- エージェントと制約の管理
- マルチスレッド実行サポート

### ソルバー

- 線形計画（LP）ソルバー
- 速度最適化アルゴリズム
- プラガブル設計

## 依存関係

### ビルド依存

- `ament_cmake_auto`
- `crane_geometry`
- `crane_msg_wrappers`
- `robocup_ssl_msgs`
- `eigen`
- `tbb` (Intel Threading Building Blocks)

### テスト依存

- `ament_cmake_catch2`

## 使用方法

### 基本的な使用例

```cpp
#include <modern_orca/simulator.hpp>
#include <modern_orca/agents/circular_agent.hpp>
#include <modern_orca/constraints/orca_constraint.hpp>

using namespace modern_orca;

// シミュレータ作成
Simulator simulator;

// エージェント追加
auto agent1 = simulator.addAgent<CircularAgent>(
    Vector2{0.0, 0.0},    // 位置
    Vector2{1.0, 0.0},    // 希望速度
    0.1,                   // 半径
    2.0                    // 最大速度
);

auto agent2 = simulator.addAgent<CircularAgent>(
    Vector2{2.0, 0.0},
    Vector2{-1.0, 0.0},
    0.1,
    2.0
);

// シミュレーション実行（60 FPS）
for (int step = 0; step < 100; ++step) {
    simulator.step(1.0 / 60.0);

    // 結果取得
    auto pos = simulator.getAgent(agent1).position();
    auto vel = simulator.getAgent(agent1).velocity();
}
```

### カスタム制約の追加

```cpp
// カスタム制約クラス
class MyConstraint : public Constraint {
public:
    auto generateHalfPlanes(const Agent& agent, TimeStep dt) const
        -> std::vector<HalfPlane> override {
        // 独自の制約ロジック
        return {HalfPlane{normal, point}};
    }
};

// 制約の登録と使用
ConstraintRegistry::register<MyConstraint>("my_constraint");
simulator.addConstraint<MyConstraint>(agent_id, /* パラメータ */);
```

### 半平面制約API

```cpp
// 直接的な速度制約の追加
simulator.addConstraint<CustomHalfPlaneConstraint>(
    agent1,
    Vector2{0.0, 1.0},    // 法線ベクトル
    Vector2{0.0, 0.5}     // 直線上の点
);
```

## CMake統合

```cmake
find_package(modern_orca REQUIRED)
target_link_libraries(your_target modern_orca::modern_orca)
```

## ROS 2統合

ROS 2での使用例は`examples/ros2_wrapper.cpp`を参照してください。

## 最近の開発状況

- **2025年前半**: C++20ベースの再設計と実装
- **設計方針**: RVO2の機能を継承しつつ、より拡張性の高いアーキテクチャを実現
- **今後の方針**: `crane_local_planner`への統合と実戦運用

## 関連パッケージ

- [crane_local_planner](./crane_local_planner.md) - このライブラリを活用する経路計画パッケージ
- [crane_geometry](./crane_geometry.md) - 幾何学計算ライブラリ（依存先）
- [rvo2_vendor](./rvo2_vendor.md) - 従来のRVO2ライブラリ

## パフォーマンス特性

| 項目 | 特徴 |
|------|------|
| **計算量** | O(n²) エージェント間の相互作用 |
| **並列化** | OpenMPによるマルチスレッド対応 |
| **メモリ** | オブジェクトプーリングによる効率化 |
| **リアルタイム性** | 60-100Hz制御ループに対応 |

## 技術仕様

### ORCA アルゴリズム

- 最適往復衝突回避アルゴリズムの実装
- 複数エージェント間の協調的な衝突回避
- 線形速度制約による効率的な計算

### 拡張性

- プラグインベースの制約追加
- カスタムエージェント形状のサポート
- ソルバーのカスタマイズ

## 備考

- ヘッダーオンリー設計のため、ビルド時間への影響が最小限
- C++20機能を活用した型安全で効率的な実装
- RVO2からの移行を容易にする互換性API
- RoboCup SSLの実機ロボット制御での使用を想定した設計

## ライセンス

MIT License
