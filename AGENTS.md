# AGENTS.md

このドキュメントは、このリポジトリでAIコーディングエージェント（例: Claude Code, GitHub Copilot Chat, OpenAI Codex CLI など）が作業する際の共通ガイダンスです。

## プロジェクト概要

Crane は RoboCup Small Size League (SSL) における自律ロボットチームのための ROS 2 ベースのAIフレームワークです。ROS 2 Jazzy を用いて、サッカーの試合で小型ロボットを制御します。

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
colcon build --packages-select crane_world_model_publisher crane_planner_plugins

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

# シナリオテスト（Python）
cd scenario_test
python3 emit_from_penalty_01.py
python3 STOP_ROBOT_SPEED.py

# 変更後はビルドと環境読込を実施
colcon build --packages-select <package_name>
source install/local_setup.bash
```

### 起動

```bash
# シミュレーション込みのメイン起動
ros2 launch crane_bringup crane.launch.py sim:=true

# 通信系のみ
ros2 launch robocup_ssl_comm comm.launch.py

# データパイプライン
ros2 launch crane_bringup data.launch.py
```

## アーキテクチャ概要

### コアコンポーネント

- crane_session_controller: 試合進行とゲーム状態管理
- crane_planner_plugins: プラグイン型の戦略プランナー
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

### コメントと言語

- すべてのソースコード内コメントは日本語で記述してください。
- 外部論文・式番号（例: 式(1), Eq.(2)）や外部API名などは必要に応じて英語併記可。
- ドキュメント・PR説明・レビューコメントも原則日本語（必要に応じて英語併記）。

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
4. 計画層: `crane_session_controller`, `crane_planner_plugins`, `crane_local_planner`
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
# 1. 余計な変更がないか
git status

# 2. 生成物がステージされていないか
git diff --cached --name-only | grep -E "(build/|install/|log/|\.idea|\.vscode|\.o$|\.so$|CMakeCache\.txt)"

# 3. あれば除外
git rm -r --cached build/ install/ log/ .idea/ .vscode/ || true
git reset HEAD -- build/ install/ log/ .idea/ .vscode/ || true

# 4. 必要なら掃除
rm -rf build/ install/ log/
```

### 緊急クリーンアップ（誤って生成物をコミットした場合）

```bash
# 直近のコミットから除外
git rm -r --cached build/ install/ log/ .idea/ .vscode/
git commit -m "Remove build artifacts and IDE settings from git tracking"

# 過去の履歴から除外（注意: 履歴を書き換えます）
FILTER_BRANCH_SQUELCH_WARNING=1 git filter-branch --force --index-filter \
  'git rm -rf --cached --ignore-unmatch build install log .idea .vscode cmake-build-* */cmake-build-*' \
  --prune-empty HEAD~20..HEAD
```

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

## ボールトラッキング（要点）

- EKF による 6次元状態推定 `[x, y, z, vx, vy, vz]`
- 空気抵抗、重力、摩擦を含むモデル
- 外れ値検出（マハラノビス距離）と可視化連携

## スキルシステム（要点）

- `SkillBase` / `SkillBaseWithState` を基盤に単純/状態付き/複合スキル
- WorldModelWrapper API でインターセプト/シュート評価などを取得

## ネットワークとSSL通信（抜粋）

```yaml
SSL-Vision:
  multicast: 224.5.23.2
  ports: [10006, 10020]

SSL-GameController:
  multicast: 224.5.23.1
  ports: [10003, 11111]

Robot Communication:
  base_ip: 192.168.20.100
  feedback_multicast: 224.5.20.100
```

### ROS 2 マルチキャスト設定（例）

```bash
export ROS_DOMAIN_ID=0
export CYCLONE_DDS_ENABLED_TRANSPORTS=udp
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastrtps_profile.xml
```

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

### SSL関連ツール（例）

```bash
go install github.com/RoboCup-SSL/ssl-go-tools/cmd/ssl-log-recorder@latest
ssl-log-recorder -port 10006 -output match.log
```

### VS Code 推奨拡張

- C/C++、ROS、Python、GitLens、Docker

### Colcon ビルド最適化（再掲）

```bash
colcon build --symlink-install --parallel-workers $(nproc)
colcon build --packages-select crane_world_model_publisher --symlink-install
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

## シミュレーション環境

### grSim（ibis-ssl 改良版の特徴）

- ドリブル力制御、物理衝突精度の向上、ペナルティエリア出力

### Docker 統合

```bash
cd docker/sim
docker compose up -d

# 接続
# - http://localhost:8081
# - http://localhost:8082
# - http://localhost:8083
```

### 実機移行のチェックポイント

1. ネットワーク設定の切替
2. 物理パラメータ調整
3. 遅延補償
4. 安全機構（緊急停止/衝突回避）
