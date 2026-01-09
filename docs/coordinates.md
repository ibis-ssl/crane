# 座標系仕様

このドキュメントでは、Craneプロジェクトで使用される座標系について説明します。

## フィールド座標系（グローバル座標系）

SSL-Visionから受信する標準的な座標系です。

### 座標軸の定義

- **X軸**: フィールドの長辺方向（ゴールライン間の方向）
- **Y軸**: フィールドの短辺方向（サイドライン間の方向）  
- **Z軸**: 地面に垂直な方向（上向きが正）

### 原点とスケール

- **原点**: フィールド中央
- **単位**: メートル（m）
- **範囲**:
  - X: ±6.0m（標準的な12m×9mフィールドの場合）
  - Y: ±4.5m（標準的な12m×9mフィールドの場合）

### チーム方向

- **自チームゴール**: 一般的にX軸の負の方向
- **相手チームゴール**: 一般的にX軸の正の方向

## ロボット座標系（ローカル座標系）

個々のロボットを基準とした座標系です。

### 座標軸の定義

- **X軸**: ロボットの前方向（キッカーがある方向）
- **Y軸**: ロボットの左方向
- **Z軸**: ロボットの上方向

### 角度表現

- **回転角θ**: Z軸周りの回転（右手座標系）
- **範囲**: -π ≤ θ ≤ π（ラジアン）
- **0度**: フィールド座標系のX軸正方向
- **正の回転**: 反時計回り

## 座標変換

### フィールド座標からロボット座標への変換

```cpp
// フィールド座標系の点をロボット座標系に変換
Point transformToRobotFrame(const Point& field_point, const RobotPose& robot_pose) {
  Point relative = field_point - robot_pose.pos;
  double cos_theta = std::cos(-robot_pose.theta);
  double sin_theta = std::sin(-robot_pose.theta);

  return Point{
    relative.x() * cos_theta - relative.y() * sin_theta,
    relative.x() * sin_theta + relative.y() * cos_theta
  };
}
```

### ロボット座標からフィールド座標への変換

```cpp
// ロボット座標系の点をフィールド座標系に変換
Point transformToFieldFrame(const Point& robot_point, const RobotPose& robot_pose) {
  double cos_theta = std::cos(robot_pose.theta);
  double sin_theta = std::sin(robot_pose.theta);

  Point rotated{
    robot_point.x() * cos_theta - robot_point.y() * sin_theta,
    robot_point.x() * sin_theta + robot_point.y() * cos_theta
  };

  return rotated + robot_pose.pos;
}
```

## 実装上の注意点

### データ型

- **Point型**: `Vector2`のエイリアス（2D座標）
- **Vector3型**: `Vector3`（3D座標、z成分含む）
- **角度**: `double`型、ラジアン単位

### 幾何学ライブラリ

座標計算には`crane_geometry`パッケージや、`RobotInfo`のメソッドを使用：

```cpp
#include <crane_geometry/geometry_operations.hpp>
#include <crane_physics/robot_info.hpp>

// 角度の正規化
double normalized_angle = crane::normalizeAngle(angle);

// 2点間距離 (Eigenの機能を使用)
double distance = (point1 - point2).norm();

// ロボットからの距離 (RobotInfoのメソッドを使用)
double dist_to_ball = robot->getDistance(ball_pos);

// 点1から点2への方向（角度）
double angle = crane::getAngle(point2 - point1);
```

## 関連ドキュメント

- [crane_geometry パッケージ](./packages/crane_geometry.md) - 幾何学計算ライブラリ
- [crane_physics パッケージ](./packages/crane_physics.md) - 物理計算と座標変換
- [SSL-Vision 仕様](https://ssl.robocup.org/ssl-vision/) - 公式座標系仕様
