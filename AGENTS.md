# AGENTS.md

このドキュメントは、このリポジトリでAIコーディングエージェント（例: Claude Code, GitHub Copilot Chat, OpenAI Codex CLI など）が作業する際の共通ガイダンスです。

## プロジェクト概要

Crane は RoboCup Small Size League (SSL) における自律ロボットチームのための ROS 2 ベースのAIフレームワークです。ROS 2 Jazzy を用いて、サッカーの試合で小型ロボットを制御します。

**システム全体構成**: Crane は3つのプログラム（crane, Orion_CM4, G474_Orion_main）が連携してロボットを制御します。Crane（AI PC）が戦略立案・経路計画を担当し、Orion_CM4（Raspberry Pi CM4）が通信ブリッジとして機能し、G474_Orion_main（STM32マイコン）がモーター制御を実行します。詳細は「システム全体アーキテクチャ」セクションを参照してください。

## ビルド・開発コマンド

### 重要: ビルド実行場所について

⚠️ 必須要件: すべてのビルドコマンドは ROS ワークスペースのルートで実行してください。

- ❌ 間違い: `<ワークスペース>/src/crane/`（このリポジトリのルート）
- ✅ 正しい: `<ワークスペース>/`（`src/`, `build/`, `install/` が並ぶ場所）

理由:

- `colcon build` はワークスペース全体の依存関係を解決するため
- `build/`, `install/`, `log/` はワークスペース直下に生成されるため
- 複数パッケージ間の依存を正しく処理するため

AIエージェントへの指示:

- ビルド前に必ずワークスペースルートへ移動すること
- ワークスペースルートにアクセスできない場合はユーザーに許可を求めること
- `src/`, `build/`, `install/` の存在を確認してからビルドすること

### 初期セットアップ

```bash
# 依存取得とワークスペース初期化
vcs import src < src/crane/dependency_jazzy.repos
rosdep install -riy --from-paths src

# 開発向けビルド（symlink install）
colcon build --symlink-install

# 環境読み込み
source install/local_setup.bash
```

### 開発ビルド

```bash
# 標準の最適化済み開発ビルド
colcon build --symlink-install

# 最適化スクリプト
./src/crane/scripts/optimized_build.bash

# クリーンビルド（最適化スクリプト）
./src/crane/scripts/optimized_build.bash clean

# CI相当（リリース+カバレッジ）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --mixin coverage-gcc coverage-pytest compile-commands

# 特定パッケージのみ
colcon build --packages-select crane_world_model_publisher crane_tactics

# ベンチマーク
./src/crane/scripts/optimized_build.bash benchmark
```

### ビルド時間最適化

- colcon.meta による並列設定と最適化
- vendor の shallow clone
- 不要警告の抑制でログ簡素化
- `scripts/optimized_build.bash` による自動化

目安:

- 最適化前: 約 7分18秒（33パッケージ）
- 最適化後: 目標 5分30秒（20–30% 削減）

詳細: `docs/logs/portal/build_optimization_guide.md`

### テスト

```bash
# すべてのテスト
colcon test --event-handlers console_cohesion+

# パッケージ指定
colcon test --packages-select crane_physics crane_sender --event-handlers console_cohesion+

# 個別テスト（正規表現）
colcon test --packages-select crane_physics --event-handlers console_cohesion+ --ctest-args -R test_ball_msg_conversion

# シナリオテスト（推奨：Makefileを使用）
# 初回のみ環境セットアップ
make scenario-test-setup

# 全テスト実行
make scenario-test

# 個別テスト実行
make scenario-test TEST=STOP_ROBOT_SPEED
make scenario-test TEST=emit_from_penalty_01

# 詳細はscenario_test/README.mdを参照

# 変更後はビルドと環境読込を実施
colcon build --packages-select <package_name>
source install/local_setup.bash
```

### 起動

```bash
# シミュレーション込みのメイン起動
ros2 launch crane_bringup crane.launch.xml sim:=true

# 通信系のみ
ros2 launch robocup_ssl_comm comm.launch.py

# データパイプライン
ros2 launch crane_bringup data.launch.py
```

## システム全体アーキテクチャ

### 3プログラム連携構成

本システムは、AI処理から低レベル制御まで3つの異なるプログラムが役割分担して動作します。

| プログラム | 実行環境 | 言語 | 主な役割 |
|-----------|---------|------|---------|
| **crane** | AI PC (Ubuntu 22.04/ROS 2 Jazzy) | C++ | 戦略立案・ビジョン処理・経路計画・意思決定 |
| **Orion_CM4** | Raspberry Pi CM4 (Linux) | C++/Python | UDP↔UART通信ブリッジ・ローカルカメラ処理 |
| **G474_Orion_main** | STM32G474 MCU (ベアメタル) | C | 500Hz制御ループ・モーター駆動・センサー処理 |

### 通信フローとデータパス

```
┌─────────────────┐
│  SSL-Vision     │ (UDP Multicast: 224.5.23.2:10006)
│  Game Controller│ (UDP Multicast: 224.5.23.1:10003)
└────────┬────────┘
         │ UDP
         ▼
┌─────────────────────────────────────────────┐
│        AI PC (crane)                        │
│  ┌──────────────────────────────────────┐   │
│  │ crane (ROS 2 Jazzy)                  │   │
│  │  - 世界モデル構築                     │   │
│  │  - 戦略立案・プレイ選択              │   │
│  │  - 経路計画・衝突回避                │   │
│  │  - ロボットコマンド生成              │   │
│  └──────────────┬───────────────────────┘   │
│                 │ ROS Topics                 │
│  ┌──────────────▼───────────────────────┐   │
│  │ crane_sender                         │   │
│  │  - RobotCommandV2 パケット生成       │   │
│  └──────────────┬───────────────────────┘   │
└─────────────────┼───────────────────────────┘
                  │ UDP (192.168.20.1xx:12345)
                  ▼
┌─────────────────────────────────────────────┐
│   Raspberry Pi CM4 (Orion_CM4)              │
│  ┌──────────────────────────────────────┐   │
│  │ UDP受信 → UART送信                    │   │
│  │  - パケット中継・バッファリング       │   │
│  │  - (ローカルカメラ処理)              │   │
│  └──────────────┬───────────────────────┘   │
└─────────────────┼───────────────────────────┘
                  │ UART (シリアル通信)
                  ▼
┌─────────────────────────────────────────────┐
│   STM32G474 MCU (G474_Orion_main)           │
│  ┌──────────────────────────────────────┐   │
│  │ RobotCommandV2 受信・デシリアライズ   │   │
│  │  - 500Hz制御ループ                   │   │
│  │  - モーター制御 (PWM出力)            │   │
│  │  - エンコーダ・IMU読み取り           │   │
│  │  - フィードバックパケット生成        │   │
│  └──────────────┬───────────────────────┘   │
└─────────────────┼───────────────────────────┘
                  │ UART (フィードバック)
                  ▼
         (フィードバック経路は上記と逆向き)
```

### 共有コード: robot_packet.h

3つのプログラムで共通のパケット定義ファイル `robot_packet.h` を使用します。

**ファイルパス**:
- crane: `crane_sender/include/crane_sender/robot_packet.h`
- Orion_CM4: `robot_packet.h` (リポジトリルート)
- G474_Orion_main: `Core/Inc/robot_packet.h`

**重要**: 現在、各プログラムで若干異なるバージョンが使用されています。

| プログラム | 制御モード | 構造体サイズ | 備考 |
|-----------|----------|-------------|------|
| crane | POLAR_VELOCITY_TARGET_MODE | 64バイト | シンプルな極座標速度指令 |
| Orion_CM4 | 複数モード対応 | 64バイト | LOCAL_CAMERA, POSITION, VELOCITY等 |
| G474_Orion_main | POLAR_VELOCITY_TARGET_MODE | 64バイト | crane と同じバージョン |

**変更時の作業手順**:
1. マスターファイル（crane版）を修正
2. 構造体の互換性を確認（サイズ、アライメント、エンディアン）
3. 3つのリポジトリすべてに同期（手動コピー）
4. 各プログラムで個別にビルド・テスト
5. 統合テストで通信確認

**変更影響範囲**:
- パケット構造の変更 → **全プログラムの再ビルド必須**
- 新フィールド追加 → **下位互換性に注意**
- エンディアン変更 → **異なるアーキテクチャ間の通信エラー発生**

### ネットワーク設定詳細

詳細なネットワーク構成図と設定方法は `docs/network.md` を参照してください。

**主要アドレス・ポート**:
- SSL-Vision: `224.5.23.2:10006` (本番) / `:10020` (grSim)
- SSL-GameController: `224.5.23.1:10003` (本番) / `:11003` (grSim)
- ロボットコマンド: `192.168.20.100+機体番号:12345`
- ロボットフィードバック: `224.5.20.100:50100+機体番号`

## アーキテクチャ概要

### コアコンポーネント

- crane_tactic_coordinator: 試合進行とゲーム状態管理
- crane_tactics: プラグイン型の戦略プランナー
- crane_robot_skills: 個別ロボット振る舞い（ゴールキーパー等）
- crane_local_planner: RVO2 ベースの局所経路計画
- crane_world_model_publisher: 状態推定とトラッキング
- crane_play_switcher: 形勢判断と自動プレイ選択

### メッセージフロー

1. `robocup_ssl_comm` がSSLの視覚/審判データを受信
2. `crane_world_model_publisher` が世界モデルを配信
3. セッションコントローラが上位意思決定
4. プランナが割当と戦略を生成
5. スキルがロボットコマンドへ変換
6. `crane_sender` がシミュレータ/実機へ送信

### 主なディレクトリ

- `session/` 上位制御と戦略
- `utility/` 共有ユーティリティ（幾何など）
- `consai_ros2/` SSL通信
- `crane_msgs/` メッセージ定義
- `3rdparty/` 依存ライブラリ

## 開発環境

### Docker（シミュレーション）

```bash
cd docker/sim
docker compose up -d

# アクセス
# - Game Controller: http://localhost:8081
# - Vision Client:   http://localhost:8082
# - Status Board:    http://localhost:8083
```

### 実機環境

```bash
cd docker/real
docker compose up -d
```

## コード規約

### ビルド

- `ament_cmake_auto` を使用
- パッケージごとに標準化した CMakeLists.txt
- 独自 lint: `crane_lint_common`
- C++20, 主なフラグ: `-Wall -Wextra -Wpedantic -g`

### テスト構成

- GTest によるユニットテスト（各パッケージ `test/`）
- Python によるシナリオ統合テスト（`scenario_test/`）
- CI で包括的テストを実行
- pre-commit による clang-format, cpplint, ruff, ROS系 lint

### メッセージ定義

- `crane_msgs`（独自メッセージ）
- `consai_ros2/robocup_ssl_msgs`（SSL プロトコル）
- `crane_visualization_interfaces`（可視化）
- Ball の 3D は `position.z` / `velocity.z` を利用

### 依存レイヤ構成（概略）

1. メッセージ層: `crane_msgs`, `robocup_ssl_msgs`, `crane_visualization_interfaces`
2. ユーティリティ層: `crane_geometry`, `crane_physics`, `crane_comm`, `crane_msg_wrappers`
3. コンポーネント層: `crane_world_model_publisher`, `crane_game_analyzer`, `crane_robot_skills`
4. 計画層: `crane_tactic_coordinator`, `crane_tactics`, `crane_local_planner`
5. 統合層: `crane_bringup`, `crane_sender`, `robocup_ssl_comm`

## Git 運用

### 重要: 絶対にコミットしないもの

- `build/`, `install/`, `log/`
- `.idea/`, `.vscode/`, `cmake-build-*`
- 生成物: `*.o`, `*.so`, `*.a`, `CMakeCache.txt`, `CMakeFiles/` など

### .gitignore 確認

```gitignore
# Build
build/
install/
log/

# IDE
.vscode/
.idea/
**/cmake-build-debug/
**/cmake-build-*/

# CMake
CMakeCache.txt
CMakeFiles/
cmake_install.cmake
*.cmake
CTestConfiguration.ini
CTestCustom.cmake
CTestTestfile.cmake

# Objects
*.o
*.obj

# Libs
*.lib
*.a
*.la
*.lo
*.so
*.so.*
*.dylib

# Execs
*.exe
*.out
*.app

# Testing
Testing/

# Ament
ament_cmake_*/
```

### Pre-commit チェック

```bash
pre-commit run --all-files
```

AIエージェントは `git status` で生成物（`build/`, `install/`, `log/`）が
ステージングされていないことを確認すること。

**注意**: 誤って生成物をコミットした場合は、人間の開発者に報告してください。

## バージョニングとリリース管理

このプロジェクトは自動バージョニング・リリースシステムを採用しています。

### 自動バージョンアップ

developブランチへのマージ時に、GitHub Actionsが自動的にpatchバージョンをインクリメントします。

**フロー**:

1. 機能開発ブランチでの作業
2. PRレビュー・承認
3. developへのマージ → 自動的に `X.Y.Z` → `X.Y.Z+1`
4. GitHub Releaseが自動作成される

### 手動バージョンアップ

minor/majorバージョンアップが必要な場合は、GitHub ActionsのUIから手動実行します。

**手順**:

1. GitHub Actions → `Auto Version Bump and Release` を選択
2. `workflow_dispatch` を実行
3. バージョンバンプタイプを選択:
   - `patch`: X.Y.Z → X.Y.Z+1（デフォルト）
   - `minor`: X.Y.Z → X.Y+1.0
   - `major`: X.Y.Z → X+1.0.0

### バージョン管理ポリシー

- **全パッケージ統一**: 32個の全パッケージを同一バージョンで管理
- **セマンティックバージョニング**: メジャー.マイナー.パッチ形式
- **Gitタグ**: 各バージョンにタグを付与（例: `1.0.0`）
- **リリースノート**: GitHub Releaseで自動生成

### AIエージェントへの指示

- バージョン番号は手動で変更しないこと
- package.xmlのバージョンは自動管理されていることを認識すること
- リリースはGitHub Actionsに任せること

## コミット規約

### 言語ポリシー

このリポジトリのコミットメッセージは日本語で記述してください。

### 形式

```text
[カテゴリ]概要（50文字以内）

詳細説明（任意、72文字で改行）
- 具体的な変更内容
- 影響範囲や理由の説明
- 必要に応じて参考情報
```

### カテゴリ（例）

- 機能追加, バグ修正, リファクタリング, ドキュメント, テスト, ビルド, CI/CD, 設定, 翻訳, クリーンアップ

### 粒度

- 1コミット=1論理変更。コード/ドキュメント/設定は分離
- 各コミットはビルド可能かつ動作する状態

### チェックリスト

- [ ] 日本語のメッセージ
- [ ] 標準カテゴリの使用
- [ ] タイトルは50文字以内で要点明確
- [ ] 生成物は含まれていない（Git 運用参照）
- [ ] 動作確認/テスト済み

## 特筆事項

### リアルタイム制約

- ロボット制御はリアルタイム制約下で動作
- RVO2 による多ロボット回避
- SSL プロトコル準拠の通信タイミング

### 座標系とボール物理

- フィールド座標は SSL 仕様
- Ball は STOPPED/ROLLING/FLYING 状態を持つ物理モデル
- `toMsg()` / `fromMsg()` によるメッセージ変換

### プラグインアーキテクチャ

- 戦略プラグイン、スキルの合成、設定駆動のパラメータ

### ドキュメント構成

- `docs/` に主要ドキュメント（日本語）
- 各パッケージの詳細ドキュメント
- `docs/logs/` に進捗ログ

### 重要ドキュメント

- `index.md`, `setup.md`, `docker.md`, `skill.md`, `coordinates.md`, `ball_tracking_system.md`,
  `world_model_wrapper.md`, `network.md`, `vision.md`, `grSim.md`

### 複数プログラムにまたがるトラブルシューティング

このセクションでは、crane/Orion_CM4/G474 の3つのプログラムが連携する際の典型的な問題と調査手順を説明します。

#### ロボットが動かない（コマンドが届かない）

**症状**: crane から指令を出しているのに、ロボットが全く動かない。

**調査手順**:

1. **crane 側**: コマンド送信を確認
   ```bash
   # crane_sender が正常に動作しているか確認
   ros2 topic echo /robot_commands
   # UDP送信ログを確認（デバッグログが有効な場合）
   ```

2. **Orion_CM4 側**: パケット受信を確認
   ```bash
   # UDP受信ログを確認
   # UART送信ログを確認
   # LEDの点滅パターンで通信状態を確認（実装による）
   ```

3. **G474 側**: UART受信とパケット解析を確認
   - LED状態を確認（点滅パターンで通信状態を判断）
   - デバッガ（ST-Link）で変数を確認
   - `robot_command.header` の値が正しいか確認

**よくある原因**:
- ネットワーク設定ミス（IPアドレス、ポート番号）
- `robot_packet.h` のバージョン不一致
- UART通信速度（ボーレート）の不一致
- パケットヘッダーの不一致

#### パケットが届かない・通信が不安定

**症状**: コマンドが時々届かない、ロボットの動きが断続的。

**調査手順**:

1. **ネットワーク設定を確認**
   ```bash
   # crane 側: 送信先IPアドレスを確認
   ros2 param get /crane_sender robot_ip_base

   # Orion_CM4 側: UDPポートで受信しているか確認
   netstat -un | grep 12345

   # パケット損失を確認
   ping 192.168.20.101  # ロボット1号機の例
   ```

2. **各プログラムのログ確認**
   - crane: `ros2 topic hz /robot_commands` でパブリッシュ頻度を確認
   - Orion_CM4: 受信パケット数とエラー数を確認
   - G474: 受信パケット数とチェックサムエラーを確認

3. **パケット互換性を確認**
   - 各プログラムの `robot_packet.h` を比較
   - 構造体サイズが一致しているか確認（64バイト）

**よくある原因**:
- ネットワークケーブルの接続不良
- スイッチングハブの設定ミス
- パケットバッファのオーバーフロー
- UART通信のバッファ不足

#### ロボットが意図しない動きをする

**症状**: コマンドは届いているが、ロボットの動きがおかしい。

**調査手順**:

1. **パケット内容を確認**
   - crane: 送信しているコマンド値をログ出力
   - G474: 受信したコマンド値をデバッガで確認

2. **制御モードを確認**
   - `control_mode` フィールドが正しく設定されているか
   - Orion_CM4 と G474 で対応している制御モードか確認

3. **座標系の一致を確認**
   - crane が送信する座標系とロボット側の解釈が一致しているか
   - 極座標 vs デカルト座標の混同がないか

**よくある原因**:
- `robot_packet.h` のバージョン不一致によるフィールドのずれ
- エンディアンの違い（通常は問題ないが、カスタム実装の場合注意）
- 制御モードの不一致
- 単位系の違い（m vs mm、rad vs deg など）

#### デバッグに役立つツール・コマンド

```bash
# ネットワークトラフィックをキャプチャ
tcpdump -i <interface> udp port 12345 -w robot_commands.pcap

# UDPパケット受信確認
nc -u -l 12345  # Orion_CM4 のIPアドレスで実行

# ログファイルの確認
tail -f /var/log/crane_sender.log
tail -f /var/log/orion_cm4.log

# ROS 2 トピックのモニタリング
ros2 topic hz /robot_commands
ros2 topic echo /robot_feedback
```

## ボールトラッキング（要点）

- EKF による 6次元状態推定 `[x, y, z, vx, vy, vz]`
- 空気抵抗、重力、摩擦を含むモデル
- 外れ値検出（マハラノビス距離）と可視化連携

## スキルシステム（要点）

- `SkillBase` / `SkillBaseWithState` を基盤に単純/状態付き/複合スキル
- WorldModelWrapper API でインターセプト/シュート評価などを取得

## ネットワークとSSL通信

詳細なネットワーク構成、アドレス、ポート設定は `docs/network.md` を参照。

**主な通信先**:
- SSL-Vision / SSL-GameController: マルチキャスト通信
- ロボット: `192.168.20.100+機体番号:12345`

## 開発ツールとワークフロー

### pre-commit

```bash
pip install pre-commit
pre-commit install

# 実行される例
# - clang-format
# - cpplint
# - ruff
# - ament_lint
```

### SSL関連ツール

SSL公式ツールは [RoboCup-SSL/ssl-go-tools](https://github.com/RoboCup-SSL/ssl-go-tools) を参照。

### VS Code 推奨拡張

- C/C++、ROS、Python、GitLens、Docker

## シミュレーション環境

### grSim（ibis-ssl 改良版の特徴）

- ドリブル力制御、物理衝突精度の向上、ペナルティエリア出力

### Docker 環境

Docker環境の構築・起動方法は「開発環境」セクションを参照。

### 実機移行のチェックポイント

1. ネットワーク設定の切替
2. 物理パラメータ調整
3. 遅延補償
4. 安全機構（緊急停止/衝突回避）

## 連携プログラムリファレンス

このセクションでは、crane と連携する他の2つのプログラムの概要と、変更時の注意点を説明します。

### Orion_CM4

**リポジトリ**: [ibis-ssl/Orion_CM4](https://github.com/ibis-ssl/Orion_CM4)

**役割**: UDP ↔ UART 通信ブリッジ、ローカルカメラ処理

**主要ファイル**:
- `robot_packet.h`: パケット定義（crane と同期が必要）
- UDP受信・UART送信を行う通信プログラム
- ローカルカメラによるボール検出（オプション機能）

**実行環境**:
- Raspberry Pi CM4 (Linux)
- Python または C++ で実装

**変更時の注意点**:
- `robot_packet.h` を変更した場合は、crane/G474 と同期が必須
- UART通信速度（ボーレート）の変更は G474 と合わせる必要あり
- パケットバッファサイズの変更は通信エラーの原因となる

参照が必要な場合:
```bash
git clone git@github.com:ibis-ssl/Orion_CM4.git
```

### G474_Orion_main

**リポジトリ**: [ibis-ssl/G474_Orion_main](https://github.com/ibis-ssl/G474_Orion_main)

**役割**: 500Hz制御ループでのモーター制御・センサー処理

**主要ファイル**:
- `Core/Inc/robot_packet.h`: パケット定義（crane と同期が必要）
- `Core/Src/main.c`: メインループと制御ロジック
- モーター制御、エンコーダ読み取り、IMU処理

**実行環境**:
- STM32G474 マイコン（ARM Cortex-M4）
- ベアメタル環境（RTOS なし、または FreeRTOS）
- STM32CubeIDE でビルド

**変更時の注意点**:
- `robot_packet.h` を変更した場合は、crane/Orion_CM4 と同期が必須
- 制御ループ周期（500Hz）の変更はモーター制御に大きく影響
- パケットデシリアライズのエンディアンに注意（Cortex-M4 はリトルエンディアン）

参照が必要な場合:
```bash
git clone git@github.com:ibis-ssl/G474_Orion_main.git
```

### 変更影響範囲マトリクス

この表は、変更内容がどのプログラムに影響するかを示します。

| 変更内容 | crane | Orion_CM4 | G474_Orion_main | 備考 |
|---------|-------|-----------|----------------|------|
| パケット構造変更 | ✅ | ✅ | ✅ | `robot_packet.h` を3つすべてに同期 |
| 新制御モード追加 | ✅ | ✅ | ✅ | 全プログラムで対応が必要 |
| 戦略ロジック変更 | ✅ | - | - | crane のみで完結 |
| 経路計画アルゴリズム変更 | ✅ | - | - | crane のみで完結 |
| モーター制御PIDゲイン変更 | - | - | ✅ | G474 のみ |
| ネットワークアドレス変更 | ✅ | ✅ | - | crane と Orion_CM4 の設定を変更 |
| UART通信速度変更 | - | ✅ | ✅ | Orion_CM4 と G474 の両方で設定 |
| ローカルカメラ処理追加 | - | ✅ | - | Orion_CM4 のみ |
| センサー追加（IMU/エンコーダ） | - | - | ✅ | G474 のみ（ただし crane でのキャリブレーション必要な場合あり） |

### 統合テスト時の確認ポイント

複数プログラムの変更を統合する際は、以下を確認してください：

1. **パケット互換性**: 各プログラムで `robot_packet.h` のバージョンが一致しているか
2. **通信確認**: crane → Orion_CM4 → G474 の通信が正常に動作するか
3. **タイミング**: 制御ループの周期やレイテンシーに影響がないか
4. **エラーハンドリング**: パケット欠損や通信エラー時の動作が適切か
