# crane_local_planner

## 概要

ロボット間の**リアルタイム経路計画と衝突回避**を担うパッケージです。RVO2（Reciprocal Velocity Obstacles）アルゴリズムを中核とし、マルチロボット環境での安全かつ効率的な経路生成を実現します。SSL競技の制約下でリアルタイム性能を重視した設計となっています。

## 主要機能

- **RVO2衝突回避**: 複数ロボット間の相互衝突回避
- **リアルタイム経路計画**: 10ms以下の高速経路生成
- **複数プランナー対応**: 状況に応じたプランナー選択
- **動的障害物対応**: 移動する敵ロボット・ボールの考慮
- **制約満足**: ロボット動力学制約の考慮

## アーキテクチャ上の役割

Craneシステムの**動作計画層**として、上位スキルからの目標位置・速度を受け取り、衝突回避を考慮した実現可能な制御コマンドを生成します。リアルタイム制約とマルチロボット協調の両立を実現する重要コンポーネントです。

## プランナー構成

### RVO2Planner（メインプランナー）

- **アルゴリズム**: Reciprocal Velocity Obstacles 2.0
- **特徴**:
  - 分散的衝突回避（各ロボットが独立判断）
  - 相互協調による最適解探索
  - 実時間性能保証
- **適用場面**: 通常のマルチロボット制御

## RVO2アルゴリズム詳細

詳細なパラメータ設定やアルゴリズムの解説については、[RVO2 Local Planner詳細ドキュメント](../rvo2_local_planner.md)を参照してください。

### 基本原理

```cpp
// 速度障害物の定義
struct VelocityObstacle {
  Vector2 apex;        // 障害物頂点
  Vector2 direction1;  // 境界方向1
  Vector2 direction2;  // 境界方向2
  double radius;        // 障害物半径
};

// 許可速度の計算
Vector2 computeAllowedVelocity(
  const Vector2& preferred_velocity,
  const std::vector<VelocityObstacle>& obstacles
) {
  // 1. 速度障害物群の統合
  // 2. 許可速度空間の計算
  // 3. 最適速度の選択
  return optimal_velocity;
}
```

### マルチロボット協調

- **責任分担**: 各ロボットが相手ロボットとの衝突回避責任を分担
- **相互予測**: 他ロボットの意図を推定した協調行動
- **デッドロック回避**: 膠着状態の自動解除機構

## 技術的特徴

### 動力学制約考慮

```cpp
// ロボット制約の定義
struct RobotConstraints {
  double max_velocity;      // 最大速度
  double max_acceleration;  // 最大加速度
  double max_angular_vel;   // 最大角速度
  double radius;           // ロボット半径
};

// 制約満足解の生成
RobotCommand generateFeasibleCommand(
  const Vector2& target_position,
  const RobotConstraints& constraints
);
```

### 予測制御

- **短期予測**: 0.5秒先までの軌道予測
- **衝突判定**: 予測軌道での衝突可能性評価
- **動的調整**: 予測結果に基づく速度調整

### 速度計画予実管理機能

速度計画の予測と実績を比較し、計画精度の評価を行う機能が追加されています（VelocityPlanTraceメッセージ連携）。

**主要機能**:

- 計画速度と実際の速度の記録
- RVO2などの速度修正の追跡（VelocityCorrection）
- 計画精度の可視化・デバッグ支援

詳細: `crane_msg_wrappers/include/crane_msg_wrappers/velocity_plan_tracker.hpp`

## 依存関係

### アルゴリズム依存

- **rvo2_vendor**: RVO2アルゴリズム実装
- **closest_point_vendor**: 最近点計算ライブラリ

### システム依存

- **crane_msg_wrappers**: メッセージ変換
- **crane_msgs**: システムメッセージ
- **diagnostic_updater**: 診断機能

## 最近の開発状況

### 2025年の主要変更

- **RVO2最適化**: 並列ベクトル化により推定時間を15%削減
- **制約処理改良**: ロボット個別の減速プロファイルを導入
- **予測精度向上**: ボール予測とリンクした逃避経路評価を実装
- **デバッグ機能拡充**: 可視化トピックをFoxglove向けに整理

### 開発活発度

🔴 **高活動**: RVO2アルゴリズムの改良、新しい制約処理、パフォーマンス最適化が継続的に行われている。特に実時間性能と衝突回避精度の両立に焦点を当てた開発が活発。

### 技術的課題と解決策

- **計算負荷**: 並列処理とアルゴリズム最適化で対応
- **デッドロック**: 確率的摂動による解除機構
- **動的環境**: 予測モデルの高精度化

## パフォーマンス指標

### リアルタイム性能

- **計算時間**: <5ms（ロボット6台）
- **更新頻度**: 100Hz
- **メモリ使用量**: <50MB

### 衝突回避性能

- **衝突回避率**: >99.5%
- **目標到達率**: >95%
- **経路効率**: 最短経路の1.2倍以内

## 将来展望

### 技術発展方向

- **学習ベース**: 強化学習による経路最適化
- **予測精度**: より長期の軌道予測
- **適応制御**: 環境変化への自動適応

---

**関連パッケージ**: [rvo2_vendor](./rvo2_vendor.md) | [crane_robot_skills](./crane_robot_skills.md) | [crane_sessions](./crane_sessions.md)
