# Crane Debug Tools ドキュメント

crane_debug_tools の包括的なドキュメントへようこそ。crane_simple_ai の現代的な代替品として、Crane ロボットサッカーシステムでのロボットスキルのテストとデバッグのための強力なコマンドライン・Web ベースツールを提供します。

## crane_debug_tools とは？

crane_debug_tools は、Qt ベースの crane_simple_ai をより柔軟で自動化可能、開発者フレンドリーなツールに置き換えるために設計された次世代デバッグ・テストスイートです。Crane ロボットサッカーシステム用のコマンドラインインターフェースと Web ベースデバッグ機能の両方を提供します。

### 主な機能

- **🖥️ コマンドラインインターフェース**: 素早いテスト用のインタラクティブ・バッチ CLI ツール
- **🌐 Web インターフェース**: 現代的なブラウザベースデバッグ（計画中）
- **🤖 マルチロボットサポート**: ロボットチーム用のネイティブ連携テスト
- **📝 シナリオテスト**: JSON ベースの自動化テストシーケンス
- **🔧 クロスプラットフォーム**: ROS 2 があるあらゆるシステムで動作
- **⚡ 高性能**: GUI 代替品と比較して最小限のリソース使用
- **🚀 自動化対応**: 完全な CI/CD 統合サポート

## クイックスタート

### インストール
```bash
# パッケージをビルド
colcon build --packages-select crane_debug_tools

# ワークスペースを読み込み
source install/local_setup.bash
```

### 基本的な使用方法
```bash
# 利用可能なスキルをリスト表示
crane_skill list

# スキルを実行
crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0

# マルチロボット実行
crane_skill multi Attacker 0,1,2

# テストシナリオを実行
crane_skill scenario scenarios/basic_test.json
```

## ドキュメントセクション

### 📖 コアドキュメント

#### [ユーザーガイド](USER_GUIDE.md)
基本操作から高度なマルチロボットシナリオまで、crane_debug_tools の使用に関するすべての側面をカバーする包括的ガイド。

**内容：**
- 開始方法とインストール
- CLI インターフェース使用法
- パラメータシステムとフォーマット
- マルチロボット連携
- シナリオテスト
- ベストプラクティスとワークフロー

#### [移行ガイド](MIGRATION_GUIDE.md)
crane_simple_ai から crane_debug_tools への移行に関するステップバイステップガイド。

**内容：**
- 機能比較
- コマンドマッピング
- ワークフロー移行
- パフォーマンス改善
- 移行問題のトラブルシューティング

### 🔧 技術ドキュメント

#### [設計概要](DESIGN_OVERVIEW.md)
crane_debug_tools の背後にあるアーキテクチャと設計決定をカバーする詳細な技術ドキュメント。

**内容：**
- システムアーキテクチャ
- 設計思想
- コンポーネント間相互作用
- パフォーマンス考慮事項
- 拡張性フレームワーク

#### [API リファレンス](API_REFERENCE.md)
crane_debug_tools を使用または拡張する開発者向けの完全な API ドキュメント。

**内容：**
- コアクラスとインターフェース
- CLI コマンドリファレンス
- パラメータシステム API
- 拡張ポイント
- ROS 2 統合詳細

### 🛠️ 実践ガイド

#### [例](EXAMPLES.md)
実践的な例と実世界のユースケースの幅広いコレクション。

**内容：**
- 基本スキルテスト
- マルチロボットシナリオ
- 自動化テストワークフロー
- 統合例
- カスタムシナリオ開発

#### [トラブルシューティング](TROUBLESHOOTING.md)
一般的な問題とその解決策に関する包括的なトラブルシューティングガイド。

**内容：**
- クイック診断
- 一般的なエラー解決
- システム依存関係
- ネットワーク設定
- パフォーマンス最適化

## アーキテクチャ概要

### システムコンポーネント

```
┌─────────────────────────────────────────────────────────────┐
│                    crane_debug_tools                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────┐    ┌─────────────────┐               │
│  │   CLI Interface │    │  Web Interface  │               │
│  │                 │    │   (Future)      │               │
│  │ • Interactive   │    │ • Browser-based │               │
│  │ • Batch Scripts │    │ • Real-time viz │               │
│  │ • Automation    │    │ • Remote access │               │
│  └─────────────────┘    └─────────────────┘               │
│           │                       │                        │
│           └───────────┬───────────┘                        │
│                       │                                    │
│  ┌─────────────────────────────────────────────────────────┤
│  │              Core Engine                                │
│  │                                                         │
│  │ • Skill Execution Management                            │
│  │ • Parameter Type Resolution                             │
│  │ • Multi-Robot Coordination                              │
│  │ • Scenario Processing                                   │
│  │ • Error Handling & Validation                           │
│  └─────────────────────────────────────────────────────────┤
│                       │                                    │
└───────────────────────┼────────────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        │           ROS 2 Layer         │
        │                               │
        │ • Action Client Communication │
        │ • Topic Subscription          │
        │ • Parameter Management        │
        │ • Node Lifecycle              │
        └───────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        │         crane_robot_skills    │
        │                               │
        │ • Skill Implementations       │
        │ • Robot Command Generation    │
        │ • World Model Integration     │
        │ • Behavior State Machines     │
        └───────────────────────────────┘
```

### 利用可能なツール

#### コマンドラインツール

1. **`crane_skill_cli`** - リアルタイムスキルテスト用インタラクティブ CLI
2. **`crane_skill`** - 自動化・スクリプト用バッチ CLI スクリプト

#### サポートされているスキル

すべての crane ロボットスキルがサポートされています：

**基本スキル：**
- `Sleep`, `Idle`, `EmplaceRobot`
- `TestMotionPosition`, `TestMotionVelocity`

**ゲームスキル：**
- `Kick`, `Receive`, `Goalie`
- `Attacker`, `SubAttacker`, `StealBall`

**フォーメーションスキル：**
- `SingleBallPlacement`, `GoalKick`
- `SimpleKickOff`, `KickOffAttack`, `KickOffSupport`

**高度なスキル：**
- `SecondThreatDefender`, `FreekickSaver`
- `PenaltyKick`, `Marker`, `Teleop`

## クイック例

### 基本スキルテスト
```bash
# シンプルなスキル実行
crane_skill run Sleep 0 duration:2.0

# ロボット配置
crane_skill run EmplaceRobot 0 target_x:1.0 target_y:2.0 target_theta:0.5

# ボール操作
crane_skill run Kick 0 target_x:3.0 target_y:1.0 kick_power:4.0
```

### マルチロボットシナリオ
```bash
# フォーメーション設定
crane_skill multi EmplaceRobot 0,1,2,3 target_x:1.0 target_y:0.0

# チーム動作の調整
crane_skill run Goalie 0
crane_skill multi Attacker 1,2,3
```

### 自動化テスト
```json
{
  "name": "Basic Test",
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {"target_x": 1.0, "target_y": 0.0},
      "delay": 2
    },
    {
      "name": "Kick",
      "robot_id": 0,
      "parameters": {"target_x": 3.0, "kick_power": 5.0},
      "delay": 1
    }
  ]
}
```

実行方法：
```bash
crane_skill scenario basic_test.json
```

## crane_simple_ai との比較

| 機能 | crane_simple_ai | crane_debug_tools |
|------|-----------------|-------------------|
| **インターフェース** | Qt5 GUI | CLI + Web（計画中） |
| **自動化** | 手動のみ | 完全なスクリプトサポート |
| **マルチロボット** | 単一ロボット | ネイティブマルチロボット |
| **パフォーマンス** | ~50-100MB メモリ | ~10-20MB メモリ |
| **プラットフォーム** | Linux のみ | クロスプラットフォーム |
| **リモートアクセス** | X11 転送が必要 | SSH フレンドリー |
| **CI/CD** | サポートなし | 完全統合 |

## ヘルプの取得

### ドキュメントナビゲーション

- **新規ユーザー**: [ユーザーガイド](USER_GUIDE.md)から始めてください
- **移行**: [移行ガイド](MIGRATION_GUIDE.md)を参照してください
- **開発者**: [API リファレンス](API_REFERENCE.md)を確認してください
- **トラブルシューティング**: [トラブルシューティングガイド](TROUBLESHOOTING.md)をご覧ください
- **例**: [例コレクション](EXAMPLES.md)を閲覧してください

### サポートリソース

- **問題**: プロジェクトリポジトリでバグ報告や機能リクエスト
- **ドキュメント**: この包括的ドキュメントセット
- **例**: 豊富な例シナリオとスクリプト
- **API リファレンス**: 完全な技術ドキュメント

### コミュニティとコントリビューション

crane_debug_tools はオープンソース Crane ロボットサッカーシステムの一部です。以下の形でのコントリビューションを歓迎します：

- バグ報告と機能リクエスト
- ドキュメント改善
- 例シナリオとユースケース
- パフォーマンス最適化
- 新ツール統合

## 今後の予定

### 現在の状況
- ✅ CLI インターフェース完成・テスト済み
- ✅ マルチロボット連携サポート
- ✅ シナリオテストフレームワーク
- ✅ 包括的ドキュメント
- ⏳ Web インターフェース（将来リリース予定）

### 将来ロードマップ
- 🔮 Web ベースリアルタイム可視化
- 🔮 拡張パフォーマンス監視
- 🔮 高度シナリオエディタ
- 🔮 ROS 2 テストフレームワーク統合
- 🔮 パラメータ最適化用機械学習統合

### 今日から始める

1. **インストール**: パッケージをビルドして読み込み
2. **試してみる**: CLI で基本スキルを実行
3. **探索**: マルチロボットシナリオをテスト
4. **自動化**: カスタムテストシナリオを作成
5. **統合**: 開発ワークフローに追加

crane_debug_tools でロボットスキルデバッグの未来へようこそ！ 🚀