# Modern ORCA ライブラリ

拡張可能な制約とプラグインサポートを備えた、最適往復衝突回避（ORCA）アルゴリズムのモダンなC++20実装です。

## 機能

- **モダンC++20**: スマートポインタ、コンセプト、レンジ、および型安全設計
- **拡張可能アーキテクチャ**: カスタム制約と派生メソッド用のプラグインシステム
- **半平面制約API**: カスタム速度制約を追加するための直接インターフェース
- **ヘッダーオンリー**: 最適なパフォーマンスのためのテンプレートベース実装
- **型安全**: コンパイル時安全性を伴う強い型付け
- **マルチスレッド**: 並列エージェント処理のためのOpenMPサポート

## クイックスタート

```cpp
#include <modern_orca/simulator.hpp>
#include <modern_orca/agents/circular_agent.hpp>
#include <modern_orca/constraints/orca_constraint.hpp>

using namespace modern_orca;

// シミュレータを作成
Simulator simulator;

// エージェントを追加
auto agent1 = simulator.addAgent<CircularAgent>(
    Vector2d{0.0, 0.0},    // 位置
    Vector2d{1.0, 0.0},    // 希望速度
    0.1,                   // 半径
    2.0                    // 最大速度
);

auto agent2 = simulator.addAgent<CircularAgent>(
    Vector2d{2.0, 0.0},    // 位置
    Vector2d{-1.0, 0.0},   // 希望速度
    0.1,                   // 半径
    2.0                    // 最大速度
);

// カスタム制約を追加
simulator.addConstraint<CustomHalfPlaneConstraint>(
    agent1,
    Vector2d{0.0, 1.0},    // 法線ベクトル
    Vector2d{0.0, 0.5}     // 直線上の点
);

// シミュレーションを実行
for (int step = 0; step < 100; ++step) {
    simulator.step(1.0 / 60.0);  // 60 FPS

    // 結果を取得
    auto pos1 = simulator.getAgent(agent1).position();
    auto vel1 = simulator.getAgent(agent1).velocity();
}
```

## アーキテクチャ

### コアコンポーネント

- **エージェント**: テンプレートベースのエージェントクラス（CircularAgent、PolygonAgent）
- **制約**: プラグインサポート付きの拡張可能制約システム
- **シミュレータ**: マルチスレッドを備えたメインシミュレーションオーケストレータ
- **ソルバー**: 速度最適化用のプラガブルLP(線形計画)ソルバー

### 拡張性

```cpp
// カスタム制約
class MyConstraint : public Constraint {
public:
    auto generateHalfPlanes(const Agent& agent, TimeStep dt) const
        -> std::vector<HalfPlane> override {
        // カスタム制約ロジックをここに記述
        return {HalfPlane{normal, point}};
    }
};

// 登録して使用
ConstraintRegistry::register<MyConstraint>("my_constraint");
simulator.addConstraint<MyConstraint>(agent_id, /* パラメータ */);
```

## ビルド

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## 統合

### CMake統合

```cmake
find_package(modern_orca REQUIRED)
target_link_libraries(your_target modern_orca::modern_orca)
```

### ROS 2統合

完全なROS 2ラッパー実装については`examples/ros2_wrapper.cpp`を参照してください。

## パフォーマンス

- **ヘッダーオンリー**: ゼロコスト抽象化とインライン化
- **SIMD**: 可能な場合のベクトル化演算
- **並列**: マルチスレッドエージェント処理
- **メモリ効率**: カスタムアロケータとオブジェクトプーリング

## ライセンス

MIT License - 詳細はLICENSEファイルを参照してください。
