# crane_basics パッケージドキュメント

crane_basicsパッケージは、RoboCup SSLプロジェクト「crane」のユーティリティ機能を提供するための基本ライブラリです。このパッケージは、ロボットサッカーシステムで必要となる様々な幾何学的計算や物理モデル、ROS 2のノード管理などの基本機能を提供します。

## 概要

crane_basicsパッケージは以下の主要機能を提供します：

- 幾何学ユーティリティ（Boost.Geometryを使用）
- ボールおよびロボットモデルと情報
- PID制御
- ROS 2ノードハンドル
- 時間関連ユーティリティ
- 通信用ユーティリティ

## 主要クラスとモジュール

### 幾何学関連

#### boost_geometry.hpp

Boost.Geometryライブラリを使用した基本的な幾何学型定義を提供します。

```cpp
namespace crane
{
namespace bg = boost::geometry;
using Vector2 = Eigen::Vector2d;
using Point = Eigen::Vector2d;
using Velocity = Eigen::Vector2d;
using Accel = Eigen::Vector2d;
using Segment = bg::model::segment<Point>;
using Polygon = bg::model::polygon<Point>;
using LineString = bg::model::linestring<Point>;
using Box = bg::model::box<Point>;
using ClosestPoint = bg::closest_point_result<Point>;
using Circle = crane::geometry::model::Circle<Point>;
using Capsule = crane::geometry::model::Capsule<Point>;

struct Pose2D
{
  Point pos;
  double theta;
};

struct Velocity2D
{
  Point linear;
  double omega;
};
}
```

#### geometry_operations.hpp

幾何学的操作の関数群を提供します：

主な関数：

- `isInBox()` - 点がボックス内にあるかを判定
- `createBox()` - 2点から矩形を作成
- `getAngle()` - ベクトルから角度を取得
- `normalizeAngle()` - 角度を-πからπの範囲に正規化
- `getAngleDiff()` - 2つの角度の差を計算
- `getIntermediateAngle()` - 2つの角度の中間を計算
- `getNormVec()` - 角度から単位ベクトルを取得
- `getVerticalVec()` - ベクトルに垂直なベクトルを計算
- `getReachTime()` - 距離、初速、加速度から到達時間を計算
- `getIntersections()` - 幾何形状間の交点を計算
- `getClosestPointAndDistance()` - 幾何形状間の最近接点と距離を計算

#### circle.hpp と capsule.hpp

Boost.Geometryと互換性のある円とカプセル形状モデルを提供します。

### ボール関連

#### ball_info.hpp

ボールの状態を表すデータ構造と機能を提供します。

```cpp
struct Ball
{
  Point pos;
  Point vel;
  bool is_curve;
  bool detected;

  bool isMoving(double threshold_velocity = 0.01) const;
  bool isStopped(double threshold_velocity = 0.01) const;
  bool isMovingTowards(const Point & p, double angle_threshold_deg = 60.0, double near_threshold = 0.2) const;
  bool isMovingAwayFrom(const Point & p, double angle_threshold_deg = 60.0, double near_threshold = 0.2) const;
};
```

#### ball_model.hpp

ボールの物理モデルとその予測に関する機能を提供します。

主な関数：

- `getFutureBallPosition()` - 減速を考慮した将来のボール位置を予測
- `generateSequence()` - 等間隔の数値シーケンスを生成
- `getBallSequence()` - 時間間隔ごとのボール位置を予測して配列を生成

#### ball_contact.hpp

ロボットとボールの接触状態を管理するためのユーティリティを提供します。

### ロボット関連

#### robot_info.hpp

ロボットの状態や識別情報を表すデータ構造を提供します。

```cpp
struct RobotIdentifier
{
  bool is_ours;
  uint8_t id;
  bool operator==(const RobotIdentifier & other) const;
  bool operator!=(const RobotIdentifier & other) const;
};

struct RobotInfo
{
  uint8_t id;
  RobotIdentifier getID() const;
  Pose2D pose;
  Velocity2D vel;
  bool available = false;
  rclcpp::Time vision_detection_stamp;
  rclcpp::Time ball_sensor_stamp;
  bool ball_sensor = false;
  bool getBallSensorAvailable(rclcpp::Time now, rclcpp::Duration interval) const;
  double getDribblerDistance() const;
  Vector2 center_to_kicker() const;
  Point kicker_center() const;
  BallContact ball_contact;
  auto geometry() { return Circle{pose.pos, 0.060}; }
  double getDistance(const Point & pos);
  double getDistance(const Pose2D & pose2d);
};
```

#### travel_time.hpp

ロボットの移動時間計算に関する機能を提供します。

主な関数：

- `getTravelTime()` - 現在速度に基づく単純な移動時間計算
- `getTravelTimeTrapezoidal()` - 台形速度プロファイルを考慮した移動時間計算（加速・定速・減速を考慮）

### 制御関連

#### pid_controller.hpp

PID制御器の実装を提供します。

```cpp
class PIDController
{
public:
  PIDController() = default;
  void setGain(double p, double i, double d, double max_int = -1.0);
  double update(double error, double dt);
private:
  double kp;
  double ki;
  double kd;
  double error_prev;
  double integral = 0.0;
  double max_integral = -1.0;
};
```

### ROS 2関連

#### node_handle.hpp

ROS 2ノードインターフェースへのアクセスを簡略化するためのユーティリティを提供します。

```cpp
template <typename... Interfaces>
class NodeHandle
{
public:
  explicit NodeHandle(std::shared_ptr<Interfaces>... interfaces);
  explicit NodeHandle(rclcpp::Node & node);

  template <typename T>
  std::shared_ptr<T> get_interface();
private:
  std::tuple<std::shared_ptr<Interfaces>...> interfaces_;
};
```

#### diagnosed_publisher.hpp

診断情報を含むROS 2パブリッシャーの拡張機能を提供します。

### その他のユーティリティ

#### time.hpp

時間関連のユーティリティ関数を提供します。

#### interval.hpp

数値区間を表すユーティリティクラスを提供します。

#### ddps.hpp

分散データ処理システム（DDPS）関連のユーティリティを提供します。

#### multicast.hpp と udp_sender.hpp

ネットワーク通信用のユーティリティクラスを提供します。

#### parameter_with_event.hpp

ROS 2パラメータ変更イベント対応ユーティリティを提供します。

#### position_assignments.hpp

ロボットのポジション割り当て関連のユーティリティを提供します。

#### stream.hpp

データストリーム処理用のユーティリティを提供します。

#### target_geometry.hpp

ターゲット位置計算用のユーティリティを提供します。

## 使用例

### PID制御器の使用

```cpp
#include <crane_basics/pid_controller.hpp>

crane::PIDController controller;
controller.setGain(1.0, 0.1, 0.01);  // P=1.0, I=0.1, D=0.01の設定

// 制御ループ内で
double dt = 0.01;  // 10ms
double error = target - current;
double control_output = controller.update(error, dt);
```

### ボール位置予測の使用

```cpp
#include <crane_basics/ball_model.hpp>

crane::Point ball_pos(1.0, 2.0);  // 現在のボール位置
crane::Point ball_vel(3.0, 0.0);  // 現在のボール速度（秒速3m、x軸正方向）

// 1秒後のボール位置を予測
crane::Point future_pos = crane::getFutureBallPosition(ball_pos, ball_vel, 1.0);

// 0.1秒間隔で2秒分のボール位置予測シーケンスを取得
auto sequence = crane::getBallSequence(2.0, 0.1, ball_pos, ball_vel);
for (const auto & [pos, time] : sequence) {
  // posは予測位置、timeは予測時間
}
```

### 幾何学演算の使用

```cpp
#include <crane_basics/geometry_operations.hpp>

// 角度の正規化
double angle = 3.5;  // ラジアン
double normalized = crane::normalizeAngle(angle);  // -π〜πの範囲に

// 2つの角度の差
double diff = crane::getAngleDiff(angle1, angle2);

// 2つの線分の交点を計算
crane::Segment seg1 = {crane::Point(0, 0), crane::Point(1, 1)};
crane::Segment seg2 = {crane::Point(0, 1), crane::Point(1, 0)};
auto intersections = crane::getIntersections(seg1, seg2);
```

### ロボット移動時間計算

```cpp
#include <crane_basics/travel_time.hpp>

auto robot = std::make_shared<crane::RobotInfo>();
robot->pose.pos = crane::Point(0, 0);
robot->vel.linear = crane::Point(1, 0);  // 1m/sでx軸正方向に移動中

crane::Point target(3, 4);  // 目標位置

// 単純な移動時間計算（現在速度で割るだけ）
double simple_time = crane::getTravelTime(robot, target);

// 台形速度プロファイルを考慮した移動時間計算
double trapezoid_time = crane::getTravelTimeTrapezoidal(robot, target, 2.0, 4.0);
// 最大加速度2.0m/s^2、最大速度4.0m/sの場合
```

## 依存関係

crane_basicsパッケージは以下の外部ライブラリに依存しています：

- Eigen3 - 行列・ベクトル計算
- Boost.Geometry - 幾何学計算
- ROS 2 (rclcpp) - ROSノード機能

さらに、以下のパッケージに依存しています：

- closest_point_vendor - 最近接点計算のためのベンダーパッケージ
- crane_msgs - クレーンプロジェクト用メッセージ定義
