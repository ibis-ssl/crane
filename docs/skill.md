# Skill実装ガイド

Skillは単一ロボットの行動を表す基本単位です。

## 基本概念

### スキルの役割

- **入力**: world_model（ワールド状態）
- **出力**: ロボットコマンド + 可視化情報
- **実装場所**: `crane_robot_skills`パッケージ

### ベースクラス選択

| ベースクラス | 用途 | 実装要件 |
|------------|------|---------|
| `SkillBase` | シンプルな行動制御 | `update()`メソッドのみ |
| `SkillBaseWithState<T>` | 複雑な状態遷移 | `addStateFunction()` + `addTransition()` |

### 基本実装パターン

#### シンプルスキル例

```cpp
// シンプルスキル例
class SimpleMove : public SkillBase
{
public:
  // コンストラクタ: ワールドモデルも受け取る必要がある
  explicit SimpleMove(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
    : SkillBase("SimpleMove", id, wm) {}

  Status update() override {
    // commander()経由でコマンド操作
    commander()->setTargetPosition(getParameter<Point>("target"));
    return Status::RUNNING;
  }
};
```

#### 状態付きスキル例

```cpp
// 状態付きスキル例  
enum class KickState { ENTRY_POINT, APPROACH, KICK, FINISH };

class Kick : public SkillBaseWithState<KickState>
{
public:
  explicit Kick(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
    : SkillBaseWithState("Kick", id, wm) {

    // 状態ごとの処理定義
    addStateFunction(KickState::ENTRY_POINT, [this]() {
       visualizer->drawDebugLabel(robot()->pose.pos, "Kick::ENTRY_POINT");
       return Status::RUNNING;
    });

    // 遷移定義
    addTransition(KickState::ENTRY_POINT, KickState::APPROACH, [this]() {
       return true; // 遷移条件
    });

    addStateFunction(KickState::APPROACH, [this]() {
      return approachBall();
    });

    // ...
  }
};
```

## APIリファレンス

### 基本メソッド

| メソッド | 戻り値 | 説明 |
|---------|-------|------|
| `commander()` | `PositionCommandWrapper::SharedPtr` | ロボットコマンド操作用ラッパー |
| `world_model()` | `WorldModelWrapper::SharedPtr` | ワールド状態取得 |
| `getID()` | `uint8_t` | ロボットID取得 |
| `robot()` | `RobotInfo::SharedPtr` | 自身のロボット情報取得 |
| `setParameter(key, value)` | `void` | パラメータ設定 |
| `getParameter<T>(key)` | `T` | パラメータ取得 |

### Status列挙型

| 値 | 意味 | 用途 |
|----|------|------|
| `SUCCESS` | 成功完了 | 目標達成時 |
| `FAILURE` | 失敗終了 | エラー発生時 |
| `RUNNING` | 実行中 | 継続実行時 |

### パラメータシステム

```cpp
// パラメータ設定
skill.setParameter("target", Point(1.0, 2.0));
skill.setParameter("max_velocity", 3.0);
skill.setParameter("enable_collision_avoidance", true);

// パラメータ取得
auto target = getParameter<Point>("target");
auto velocity = getParameter<double>("max_velocity");
```

### 可視化

`crane::VisualizerMessageBuilder`を使用します。`SkillBase`が自動的にインスタンスを管理します。

```cpp
// VisualizerMessageBuilderを使用
visualizer->drawCircle(center, radius, "blue");
visualizer->drawDebugLabel(robot()->pose.pos, "Status: Running");
visualizer->arrow(start, end, "white");
```

## 主要スキル一覧

### 攻撃系

- **Attacker**: メイン攻撃ロボット
- **SubAttacker**: 攻撃サポート
- **Kick**: ボールキック実行
- **Receive**: パス受け取り
- **PenaltyKick**: ペナルティキック実行

### 守備系  

- **Goalie**: ゴールキーパー専用
- **SecondThreatDefender**: 守備
- **Marker**: 敵ロボットマーク

### ユーティリティ・その他

- **Idle**: 待機
- **Teleop**: 手動操作
- **Sleep**: 待機状態
- **BallNearbyPositioner**: ボール近傍位置合わせ
- **GoalKick**: ゴールキック
- **SimpleKickoff**: キックオフ
- **CenterStopKick**: センターサークルでの停止・キック
- **EmplaceRobot**: 指定位置へのロボット配置
- **SingleBallPlacement**: ボールプレースメント
- **BallCalibrationDataCollector**: ボールモデル学習用データ収集

## 統合手順

### 1. SimpleAIタクティックでの使用

```cpp
// SimpleAIタクティックでの使用
auto skill = std::make_shared<YourSkill>(robot_id, world_model);
skill->setParameter("target", target_position);
// 実行
skill->run();
// コマンド取得
auto cmd = skill->getRobotCommand();
```

### 2. セッション統合

統一設定ファイルに追加：

```yaml
# config/unified_session_config.yaml
situations:
  YOUR_SITUATION:
    sessions:
      - name: your_skill_tactic
        capacity: 1
```

## ベストプラクティス

### パフォーマンス

- **ヘッダー最小化**: 実装はcppファイルに記述
- **早期リターン**: 条件チェックを先頭で実行
- **メモリ効率**: 不要な計算は避ける

### 品質保証

- **エラーハンドリング**: 異常状態での適切なSTATUS返却
- **テスタビリティ**: 単体テストを意識した設計
- **可視化**: デバッグ情報の適切な出力

### 設計原則

- **単一責任**: 1つのスキル = 1つの行動
- **状態管理**: 複雑な場合は状態遷移を明確化
- **再利用性**: 他のスキルから呼び出し可能な設計

## トラブルシューティング

### よくあるエラー

| エラー | 原因 | 解決方法 |
|-------|------|---------|
| コンパイルエラー | ヘッダーの循環依存 | 前方宣言を使用 |
| 実行時エラー | パラメータ未設定 | getParameterにデフォルト値設定 |
| 動作不良 | 状態遷移ミス | ログ出力で状態確認 |

### デバッグテクニック

```cpp
// ログ出力
RCLCPP_INFO(logger_, "Skill %s: State=%d, Target=(%.2f,%.2f)",
           name.c_str(), static_cast<int>(getCurrentState()),
           target.x(), target.y());

// 可視化デバッグ
visualizer->drawDebugLabel(robot()->pose.pos, fmt::format("{}: {}", name, state_string));
```

## 関連ドキュメント

- **[crane_robot_skills](./packages/crane_robot_skills.md)**: パッケージ詳細
- **[coordinates.md](./coordinates.md)**: 座標系仕様  
- **[visualizer.md](./visualizer.md)**: 可視化API
- **[session/crane_tactic_coordinator](./packages/crane_tactic_coordinator.md)**: セッション統合
