# シナリオテスト

このディレクトリには、Craneの動作を検証するシナリオテストが含まれています。

## 概要

シナリオテストは、grSim（シミュレータ）とautoref（自動審判）を使用して、実際のゲームシナリオでCraneの動作を検証します。pytestフレームワークと[robocup_scenario_test](https://github.com/SSL-Roots/robocup_scenario_test)ライブラリを使用しています。

## テスト一覧

| テスト名 | 説明 |
|---------|------|
| `STOP_ROBOT_SPEED` | STOP状態でのロボット速度制限テスト（1.5m/s以下） |
| `emit_from_penalty_01` | ペナルティエリアからのボール排出テスト |
| `STOP_AVOID_BALL` | STOP状態でのボール回避テスト（0.4m以上離れる） |
| `PENALTY_AREA_BYPASS_STABILITY` | ペナルティ横断回避時の侵入防止・安定性テスト |

## 前提条件

### 必須

- Docker（grSimとautorefを実行するため）
- Python 3.12以上
- `protobuf-compiler`（自動インストールされます）

### ローカルモード（デフォルト）の場合

- ワークスペースが既にビルドされていること
- ベースDockerイメージ：`ghcr.io/ibis-ssl/crane:base`

### リモートモードの場合

- シナリオテスト用Dockerイメージがビルドされていること

## セットアップ

### 初回のみ実行

```bash
# リポジトリルートで実行
make scenario-test-setup
```

このコマンドは以下を実行します：

- Python仮想環境の作成（`scenario_test_env/`）
- 必要なライブラリのインストール（robocup_scenario_test、pytestなど）
- ローカルモードの場合：セットアップ完了
- リモートモードの場合：Dockerイメージのビルドも実行

## テストの実行

### 基本的な使い方

```bash
# 全テストを実行
make scenario-test

# 個別テストを実行
make scenario-test TEST=STOP_ROBOT_SPEED
make scenario-test TEST=emit_from_penalty_01
```

### ローカルモード vs リモートモード

#### ローカルモード（デフォルト、推奨）

ローカルでビルドしたワークスペースを使用します。開発中のコード変更がすぐに反映されます。

```bash
# デフォルトでローカルモード
make scenario-test

# 明示的にローカルモードを指定
USE_LOCAL=1 make scenario-test
```

**メリット：**

- コード変更が即座に反映される
- Dockerイメージのビルドが不要（ベースイメージのみ）
- TDDサイクルが高速

**前提条件：**

- ワークスペースルート（`ibis_ws_3/`）で`colcon build`が完了していること
- ベースイメージ（`ghcr.io/ibis-ssl/crane:base`）がpullされていること

#### リモートモード

Dockerイメージ内でビルドされたCraneを使用します。CIと同じ環境でテストしたい場合に使用します。

```bash
# リモートモードで実行
USE_LOCAL=0 make scenario-test

# カスタムイメージタグを指定
USE_LOCAL=0 CRANE_TAG=my-custom-tag make scenario-test
```

**メリット：**

- CIと同じ環境で検証できる
- ローカルビルドの状態に依存しない

**前提条件：**

- Dockerイメージがビルドされていること（`make scenario-test-build`）

## 高度な使い方

### Docker環境の手動制御

テストを繰り返し実行する場合、Docker環境を起動したままにすることで高速化できます。

```bash
# Docker環境を起動
make scenario-test-docker-up

# テストを実行（Docker環境は起動済み）
# ※ このケースでは、スクリプトを直接使用
source scenario_test_env/bin/activate
pytest scenario_test/STOP_ROBOT_SPEED.py --vision_port=10020

# Docker環境を停止
make scenario-test-docker-down
```

### 生成されるファイル

- `*.log.gz`: 通信ログ
- `*.mp4`: テスト失敗時の動画

### 環境のクリーンアップ

```bash
# Python仮想環境、Dockerイメージ、ログファイルなどを削除
make scenario-test-clean
```

## トラブルシューティング

### Python仮想環境が見つからない

```text
エラー: Python仮想環境が見つかりません
```

**解決方法：**

```bash
make scenario-test-setup
```

### ベースイメージが見つからない（ローカルモード）

```text
Error response from daemon: pull access denied for ghcr.io/ibis-ssl/crane:base
```

**解決方法：**

```bash
docker pull ghcr.io/ibis-ssl/crane:base
```

### ワークスペースがビルドされていない（ローカルモード）

```text
bash: ../install/setup.bash: No such file or directory
```

**解決方法：**

```bash
# ワークスペースルートでビルド
cd /path/to/ibis_ws_3
colcon build --symlink-install
```

### テストがタイムアウトする

Docker環境の起動に時間がかかる場合があります。`run_test.sh`の待機時間を調整してください。

```bash
# run_test.sh の該当箇所
sleep 5  # 必要に応じて増やす
```

## CI/CDとの互換性

このローカルテスト環境は、GitHub Actions（`.github/workflows/scenario_test.yaml`）と互換性があります。CIでは以下の違いがあります：

- CIでは常にリモートモード（Dockerイメージをビルドして使用）
- CIでは全テストが並列実行される
- CIではテスト失敗時に自動的に動画とログがアーティファクトとしてアップロードされる

## 開発ワークフロー

### TDD（テスト駆動開発）の例

1. **テストを実行して失敗を確認**

   ```bash
   make scenario-test TEST=STOP_ROBOT_SPEED
   ```

2. **コードを修正**
   エディタで`crane_tactics/`などを編集

3. **ワークスペースを再ビルド**

   ```bash
   cd /path/to/ibis_ws_3
   colcon build --symlink-install --packages-select crane_tactics
   ```

4. **テストを再実行**

   ```bash
   make scenario-test TEST=STOP_ROBOT_SPEED
   ```

5. **成功するまで2-4を繰り返す**

6. **全テストを実行して回帰がないか確認**

   ```bash
   make scenario-test
   ```

### デバッグワークフロー

1. **テスト実行（ログは自動記録、失敗時は動画も自動生成）**

   ```bash
   make scenario-test TEST=STOP_ROBOT_SPEED
   ```

2. **テスト失敗時：生成された動画を確認**

   ```bash
   ls -lh *.mp4
   # vlcなどで再生
   vlc *.mp4
   ```

3. **ログを解析**

   ```bash
   gunzip -c *.log.gz > test.log
   # ログを解析ツールで確認
   ```

## エージェント向けの推奨事項

このテスト環境は、AIエージェントがテスト駆動開発を行いやすいように設計されています。

### 推奨ワークフロー

1. **初回セットアップ**

   ```bash
   make scenario-test-setup
   ```

2. **機能実装前にテストを実行（Red）**

   ```bash
   make scenario-test TEST=<新機能のテスト>
   ```

3. **機能を実装（Green）**
   - コードを編集
   - ワークスペースを再ビルド

4. **テストを実行して成功を確認**

   ```bash
   make scenario-test TEST=<新機能のテスト>
   ```

5. **リファクタリング**
   - コードを改善
   - テストで回帰がないことを確認

6. **全テストで回帰テスト**

   ```bash
   make scenario-test
   ```

### ヒント

- **高速フィードバック**: ローカルモードは変更がすぐに反映されるため、TDDサイクルが高速です
- **詳細なデバッグ**: `LOGGING=1 VIDEO=1`を使用すると、テスト失敗時の詳細な情報が得られます
- **個別テスト実行**: 開発中は`TEST=<テスト名>`で個別テストのみを実行すると効率的です

## 参考リンク

- [robocup_scenario_test](https://github.com/SSL-Roots/robocup_scenario_test): シナリオテストライブラリ
- [grSim](https://github.com/RoboCup-SSL/grSim): SSLシミュレータ
- [ssl-go-tools](https://github.com/RoboCup-SSL/ssl-go-tools): SSL関連ツール（ログレコーダーなど）
