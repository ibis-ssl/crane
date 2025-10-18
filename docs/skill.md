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
| `SkillBaseWithState<T>` | 複雑な状態遷移 | `update()` + 状態定義 |

### 基本実装パターン

```cpp
// シンプルスキル例
class SimpleMove : public SkillBase
{
public:
  explicit SimpleMove(uint8_t id) : SkillBase("SimpleMove", id) {}

  Status update() override {
    auto command = getCommand();
    command.setTargetPosition(target_position);
    return Status::RUNNING;
  }
};

// 状態付きスキル例  
enum class KickState { APPROACH, KICK, FINISH };

class Kick : public SkillBaseWithState<KickState>
{
public:
  explicit Kick(uint8_t id) : SkillBaseWithState("Kick", id) {
    setCurrentState(KickState::APPROACH);
  }

  Status update() override {
    switch(getCurrentState()) {
      case KickState::APPROACH: return approachBall();
      case KickState::KICK: return executekick();
      case KickState::FINISH: return Status::SUCCESS;
    }
  }
};
```

## APIリファレンス

### 基本メソッド

| メソッド | 戻り値 | 説明 |
|---------|-------|------|
| `getCommand()` | `RobotCommandWrapper&` | ロボットコマンド取得 |
| `getWorld()` | `WorldModelWrapper::SharedPtr` | ワールド状態取得 |
| `getID()` | `uint8_t` | ロボットID取得 |
| `emplace<T>(args...)` | `void` | パラメータ設定 |

### Status列挙型

| 値 | 意味 | 用途 |
|----|------|------|
| `SUCCESS` | 成功完了 | 目標達成時 |
| `FAILURE` | 失敗終了 | エラー発生時 |
| `RUNNING` | 実行中 | 継続実行時 |

### パラメータシステム

```cpp
// パラメータ設定
skill.emplace<Point>("target", Point(1.0, 2.0));
skill.emplace<double>("max_velocity", 3.0);
skill.emplace<bool>("enable_collision_avoidance", true);

// パラメータ取得
auto target = getParameter<Point>("target");
auto velocity = getParameter<double>("max_velocity");
```

### コンテキスト管理

```cpp
// コンテキスト設定
context.goal_pose = Pose2D(3.0, 0.0, 0.0);
context.keep_control = true;

// 可視化
addCircle(center, radius, color);
addText(position, "Status: Running");
```

## 主要スキル一覧

### 攻撃系

- **Attacker**: メイン攻撃ロボット（状態遷移型）
- **Kick**: ボールキック実行
- **Receive**: パス受け取り

### 守備系  

- **Goalie**: ゴールキーパー専用
- **Marker**: 敵ロボットマーク
- **Steal**: ボール奪取

### ユーティリティ

- **SimpleMove**: 基本移動
- **Sleep**: 待機状態
- **Teleop**: 手動操作

## 統合手順

### 1. SimpleAI統合

```cpp
// SimpleAIプランナーでの使用
auto skill = std::make_shared<YourSkill>(robot_id);
skill->emplace<Point>("target", target_position);
assigned_robots[robot_id] = skill;
```

### 2. セッション統合

```yaml
# config/play_situation/YOUR_SITUATION.yaml
nodes:
  - name: your_skill_planner
    type: SkillPlannerBase  
    params:
      skill_name: "YourSkill"
```

### 3. 可視化統合

```cpp
// スキル内での可視化
addCircle(target_pos, 0.1, "blue");
addText(robot_pos, skill_name + ": " + status_text);
addLine(robot_pos, target_pos, "green");
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
           getName().c_str(), static_cast<int>(getCurrentState()),
           target.x(), target.y());

// 可視化デバッグ
addText(getPosition(), fmt::format("{}: {}", getName(), getStatusString()));
```

## 関連ドキュメント

- **[crane_robot_skills](./packages/crane_robot_skills.md)**: パッケージ詳細
- **[coordinates.md](./coordinates.md)**: 座標系仕様  
- **[visualizer.md](./visualizer.md)**: 可視化API
- **[session/crane_session_controller](./packages/crane_session_controller.md)**: セッション統合
