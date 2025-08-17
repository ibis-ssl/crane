# KickerModel - キッカーモデル統合システム

## 概要

KickerModelは、ロボットのキック力とボールの初速度・停止距離の変換を行う高度なキッカーモデルシステムです。従来の線形補間による計算に加えて、BallPhysicsModelとの統合により、物理学的に正確なキック計算を提供します。

## 主な機能

### 1. 双方向変換

- **キック力 → ボール初速度**: ロボットのキック力からボールの初速度を予測
- **ボール初速度 → キック力**: 目標とするボール初速度から必要なキック力を計算

### 2. 停止距離制御

- **停止距離指定キック**: 指定した距離でボールを停止させるキック力を計算
- **停止距離予測**: キック力からボールの停止距離を予測

### 3. 高精度計算

- **BallPhysicsModel統合**: 物理モデルを使用した高精度な軌道計算
- **YAML設定管理**: 設定ファイルによる柔軟なパラメータ管理

## 基本的な使用方法

### 1. KickerModelの作成

```cpp
#include "crane_physics/kicker_model.hpp"

// デフォルト設定で作成
auto kicker_model = createDefaultKickerModel();

// YAML設定ファイルから作成
auto kicker_model = createKickerModelFromYAML("config/kicker_physics.yaml");

// BallPhysicsModelと統合して作成
auto ball_physics = std::make_shared<BallPhysicsModel>();
auto kicker_model = createIntegratedKickerModel("config/kicker_physics.yaml", ball_physics);
```

### 2. 基本的なキック計算

```cpp
// キック力からボール初速度を予測
double kick_power = 0.7;  // キック力 (0.0-1.0)
double predicted_speed = kicker_model->predictStraightKickSpeed(kick_power);
std::cout << "予測初速度: " << predicted_speed << " m/s" << std::endl;

// 目標初速度からキック力を計算
double target_speed = 4.0;  // 目標初速度 (m/s)
double required_power = kicker_model->calculateStraightKickPower(target_speed);
std::cout << "必要キック力: " << required_power << std::endl;
```

### 3. 停止距離制御（BallPhysicsModel連携）

```cpp
// BallPhysicsModelを設定
auto ball_physics = std::make_shared<BallPhysicsModel>();
kicker_model->setBallPhysicsModel(ball_physics);

// 指定距離で停止するキック力を計算
double target_stop_distance = 3.0;  // 目標停止距離 (m)
double kick_power = kicker_model->calculateKickPowerForStopDistance(target_stop_distance);
std::cout << "3m地点で停止するキック力: " << kick_power << std::endl;

// キック力から停止距離を予測
double predicted_distance = kicker_model->predictStopDistance(0.6);
std::cout << "予測停止距離: " << predicted_distance << " m" << std::endl;
```

## RobotCommandWrapperとの統合

RobotCommandWrapperでKickerModelを使用することで、直感的なキック指定が可能になります。

### 1. KickerModelの設定

```cpp
#include "crane_msg_wrappers/robot_command_wrapper.hpp"

// RobotCommandWrapperの作成
auto command = RobotCommandWrapper("skill_name", robot_id, world_model);

// KickerModelを設定
auto kicker_model = createKickerModelFromYAML("config/kicker_physics.yaml");
command.setKickerModel(kicker_model);
```

### 2. 新しいキックメソッドの使用

```cpp
// 停止距離指定キック
command.kickStraightToStopAt(2.5)  // 2.5m地点で停止
       .setTargetPosition(target_pos);

// 初速度直接指定キック
command.kickStraightWithInitialSpeed(5.0)  // 5.0 m/s の初速度
       .setTargetPosition(target_pos);

// 停止距離予測
double predicted_distance = command.predictStraightKickStopDistance(4.0);
std::cout << "4.0 m/s初速度の停止距離: " << predicted_distance << " m" << std::endl;
```

## YAML設定ファイル

### 設定ファイルの構造

```yaml
# kicker_physics.yaml
kicker_model:
  # ストレートキック設定
  straight_kick_powers:  [0.0, 0.25, 0.6, 0.9]
  straight_kick_speeds:  [0.0, 2.0, 4.0, 6.0]

  # チップキック設定
  chip_kick_powers:      [0.0, 0.5, 0.75, 1.0]
  chip_kick_distances:   [0.0, 0.3, 1.0, 2.5]

ball_physics_model:
  deceleration: 0.7      # ボール減速度 (m/s²)
  gravity: -9.81         # 重力加速度 (m/s²)
  air_resistance: 0.0    # 空気抵抗係数
  height_threshold: 0.05 # 飛行判定閾値 (m)
  speed_threshold: 0.1   # 移動判定閾値 (m/s)
  stop_threshold: 0.05   # 停止判定閾値 (m/s)
```

### パラメータ調整のガイドライン

1. **ストレートキック配列**:
   - `straight_kick_powers`: キック力の配列 (0.0-1.0)
   - `straight_kick_speeds`: 対応するボール初速度 (m/s)

2. **チップキック配列**:
   - `chip_kick_powers`: キック力の配列 (0.0-1.0)
   - `chip_kick_distances`: 対応する飛行距離 (m)

3. **実機調整**:
   - grSim環境と実機環境では値が異なる場合があります
   - 実際のロボットでテストを行い、配列を調整してください

## crane_local_plannerとの統合

crane_local_plannerは自動的にKickerModelを使用します：

```python
# launch.py での設定
{
    "kicker_physics_config": "/path/to/kicker_physics.yaml"
}
```

## 使用例

### 例1: パス用のキック

```cpp
// 3m先の味方ロボットにパス（2.5m地点で停止）
command.kickStraightToStopAt(2.5)
       .lookAt(teammate_position)
       .setTargetPosition(ball_position);
```

### 例2: シュート用のキック

```cpp
// 高速シュート（6.0 m/s の初速度）
command.kickStraightWithInitialSpeed(6.0)
       .lookAt(goal_center)
       .setTargetPosition(ball_position);
```

### 例3: 精密キック

```cpp
// ボール物理モデルを考慮した精密な距離制御
auto ball_physics = BallPhysicsModelFactory::createWithYAMLConfig("config/ball_physics.yaml");
kicker_model->setBallPhysicsModel(ball_physics);

double precise_kick_power = kicker_model->calculateKickPowerForStopDistance(1.8);
command.kickStraight(precise_kick_power);
```

## エラーハンドリング

```cpp
try {
    auto kicker_model = createKickerModelFromYAML("config/kicker_physics.yaml");
    double kick_power = kicker_model->calculateKickPowerForStopDistance(3.0);
} catch (const std::runtime_error& e) {
    // 設定ファイル読み込みエラーまたは計算エラー
    std::cerr << "KickerModel error: " << e.what() << std::endl;
    // フォールバック処理
}
```

## パフォーマンス考慮事項

1. **初期化**: KickerModelの初期化は一度だけ行い、再利用してください
2. **BallPhysicsModel統合**: 高精度計算が必要な場合のみBallPhysicsModelを設定
3. **キャッシュ**: 頻繁に使用する値は事前計算してキャッシュすることを推奨

## トラブルシューティング

### よくある問題

1. **「KickerModelが設定されていません」エラー**
   - RobotCommandWrapperに`setKickerModel()`でKickerModelを設定してください

2. **YAML読み込みエラー**
   - ファイルパスが正しいか確認してください
   - YAML形式が正しいか確認してください

3. **計算精度の問題**
   - 実機環境でのキャリブレーションが必要な場合があります
   - 配列の値を実測値に基づいて調整してください

## 今後の拡張予定

- [ ] 非線形補間サポート
- [ ] 機械学習ベースのキック力予測
- [ ] 環境条件（温度、湿度）の考慮
- [ ] リアルタイムキャリブレーション機能