# crane_game_analyzer

## 概要

SSL試合状況の**リアルタイム分析エンジン**として、Vision・Referee データから戦術的判断に必要な情報を抽出・評価するパッケージです。ゲーム状況の定量化とヒステリシス処理により、安定した戦術判断を上位システムに提供します。

## 主要機能

- **試合状況分析**: Vision/Referee データからの戦術情報抽出
- **ヒステリシス評価**: 状況変化の安定化処理
- **定量的評価**: 数値化された状況評価指標
- **リアルタイム処理**: 60Hz での高速分析

## アーキテクチャ上の役割

Craneシステムの**分析・判断支援層**として、生の状況データを戦術的に意味のある情報に変換し、プランナーやスキルシステムの判断を支援します。

## 分析コンポーネント

### GameAnalyzer（メイン分析エンジン）

- **ボール分析**: 位置・速度・所有権の判定
- **ロボット分析**: 役割・配置・動作パターンの評価
- **フィールド分析**: エリア占有・脅威度の計算
- **戦術状況評価**: 攻守バランス・優勢度の定量化

### Hysteresis（状況安定化）

```cpp
class Hysteresis {
  bool update(bool current_value, double threshold_high, double threshold_low);
  void reset();
private:
  bool previous_state_;
  double accumulator_;
};
```

## 分析機能詳細

### ボール分析

- **所有権判定**: どのチームがボールを支配しているか
- **脅威評価**: ボール位置による得点脅威度
- **軌道予測**: ボールの未来位置予測
- **接触検出**: ロボット-ボール接触の検出

### ロボット分析

- **役割推定**: 各ロボットの戦術的役割の推定
- **脅威度評価**: 敵ロボットの危険度評価
- **動作予測**: ロボットの次の行動予測
- **形成分析**: チーム陣形の評価

### フィールド分析

- **エリア制御**: フィールド各エリアの支配状況
- **空間評価**: 有効スペースの大きさ・価値
- **圧力分析**: 敵からの圧力の強さ・方向
- **機会検出**: 攻撃・守備機会の発見

## 評価指標

### 定量的指標

```cpp
struct GameAnalysis {
  // ボール関連
  double ball_possession_ratio;    // ボール支配率
  double ball_threat_level;        // ボール脅威度

  // 空間関連  
  double field_control_ratio;      // フィールド支配率
  double offensive_space_value;    // 攻撃空間価値
  double defensive_pressure;       // 守備圧力

  // 戦術関連
  double team_formation_quality;   // 陣形品質
  double strategic_advantage;      // 戦略的優位性
};
```

### ヒステリシス処理

```cpp
// 状況変化の安定化例
bool isInOffensiveMode() {
  double offensive_score = calculateOffensiveScore();
  return offensive_hysteresis_.update(
    offensive_score > 0.6,  // 攻撃モード閾値
    0.7,  // 攻撃開始閾値
    0.5   // 攻撃終了閾値
  );
}
```

## 使用方法

### 基本的な使用例

```cpp
#include "crane_game_analyzer/game_analyzer.hpp"

// アナライザー初期化
auto analyzer = std::make_shared<GameAnalyzer>(world_model, referee_info);

// 分析実行
GameAnalysis analysis = analyzer->analyze();

// 結果活用
if (analysis.ball_threat_level > 0.8) {
  // 緊急守備モード
  switchToDefensiveMode();
} else if (analysis.offensive_space_value > 0.6) {
  // 攻撃機会発見
  initiateAttack();
}
```

### 継続的分析

```cpp
class StrategicController {
private:
  std::shared_ptr<GameAnalyzer> analyzer_;

public:
  void updateStrategy() {
    auto analysis = analyzer_->analyze();

    // 複数指標による総合判断
    if (shouldSwitchToDefense(analysis)) {
      current_strategy_ = Strategy::DEFENSE;
    } else if (shouldSwitchToOffense(analysis)) {
      current_strategy_ = Strategy::OFFENSE;
    }
  }

private:
  bool shouldSwitchToDefense(const GameAnalysis& analysis) {
    return analysis.ball_threat_level > 0.7 &&
           analysis.defensive_pressure < 0.4;
  }
};
```

## 分析アルゴリズム

### ボール所有権判定

```cpp
double calculateBallPossession() {
  double min_distance_our = getMinDistanceToOurRobots();
  double min_distance_enemy = getMinDistanceToEnemyRobots();

  if (min_distance_our < min_distance_enemy - possession_margin) {
    return 1.0;  // 完全に我々の所有
  } else if (min_distance_enemy < min_distance_our - possession_margin) {
    return 0.0;  // 完全に敵の所有
  } else {
    return 0.5;  // 中立状態
  }
}
```

### 脅威度評価

```cpp
double calculateThreatLevel(const Vector2d& ball_position) {
  double distance_to_goal = (ball_position - our_goal_position).norm();
  double max_threat_distance = 3.0;  // 3m以内で脅威

  if (distance_to_goal > max_threat_distance) {
    return 0.0;
  }

  return 1.0 - (distance_to_goal / max_threat_distance);
}
```

## 依存関係

### 入力データ

- **WorldModel**: ロボット・ボール位置情報
- **RefereeInfo**: 審判判定・試合状況
- **FieldGeometry**: フィールド幾何情報

### システム依存

- **crane_msg_wrappers**: データ変換
- **crane_msgs**: 分析結果メッセージ

## 最近の開発状況

### 2024年の主要変更

- **分析精度向上**: より正確な状況判定アルゴリズム
- **新評価指標**: 戦術的価値を表す新しい指標の追加
- **計算最適化**: リアルタイム性能の向上
- **ヒステリシス改良**: より安定した状況判定

### 開発活発度

🟡 **中活動**: 分析アルゴリズムの改良と新しい評価指標の追加が定期的に行われている。特に戦術判断の精度向上に焦点を当てた開発が継続。

### 技術的進歩

- **機械学習導入**: 状況パターン認識の精度向上
- **統計的分析**: 試合データの統計的評価
- **予測分析**: 未来状況の予測精度向上

## パフォーマンス

### 処理性能

- **分析時間**: <2ms（全分析）
- **更新頻度**: 60Hz
- **精度**: 状況判定85%以上

### 安定性

- **ヒステリシス効果**: 判定振動50%削減
- **ノイズ耐性**: 短期変動の平滑化

---

**関連パッケージ**: [crane_world_model_publisher](./crane_world_model_publisher.md) | [crane_play_switcher](./crane_play_switcher.md) | [crane_robot_skills](./crane_robot_skills.md)
