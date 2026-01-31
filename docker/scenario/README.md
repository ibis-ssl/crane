# シナリオテスト環境（docker/scenario）

CIで実行される自動シナリオテスト用のDocker環境です。

## 概要

この環境は以下の特徴があります：

- **ヘッドレス実行** - GUIなしでシミュレータ・Game Controllerを実行
- **再現性** - 同じシナリオで一貫した結果を得られる
- **CI統合** - GitHub Actionsから自動実行可能
- **自己完結** - crane本体もDocker内で実行

## 使用されるシナリオ

シナリオテストの定義は `crane_planner_plugins/test/scenario_test/` にあります：

- 基本動作テスト
- スキル実行テスト
- 戦術パターンテスト

詳細は [`crane_planner_plugins/test/scenario_test/README.md`](../../crane_planner_plugins/test/scenario_test/README.md) を参照してください。

## ローカル実行

### Dockerfileの使用

```bash
cd docker/scenario
docker build -t crane:scenario-local -f Dockerfile ../..
```

このDockerfileは：

1. ROS 2 Jazzy環境をセットアップ
2. 依存パッケージをインストール
3. craneワークスペースをビルド
4. シナリオテスト環境を構築

### CI再現

GitHub ActionsのCI環境を再現する場合：

```bash
# CIと同じ手順でビルド・テスト
cd docker/scenario
./run_scenario_test_local.sh  # （スクリプトが存在する場合）
```

## CI統合

このDockerfileは以下のGitHub Actionsワークフローで使用されます：

- `.github/workflows/ros-ci.yaml` - ROS 2 CI（シナリオテスト含む）

### CIでの実行フロー

1. Dockerイメージをビルド
2. `colcon test` でシナリオテストを実行
3. テスト結果をartifactとして保存
4. 失敗時はログを出力

## テスト結果の確認

### ローカル

```bash
# colcon testの結果を確認
cat install/crane_planner_plugins/share/crane_planner_plugins/test_results/crane_planner_plugins/scenario_test.xml
```

### CI

GitHub ActionsのArtifactsセクションから以下をダウンロード：

- `test-results` - JUnit XML形式のテスト結果
- `test-logs` - 詳細ログ

## Docker環境の構成

### ベースイメージ

- `ros:jazzy-ros-base-noble` - ROS 2 Jazzy公式イメージ

### 追加パッケージ

- システムツール: git, build-essential, cmake等
- ROS依存: eigen3, protobuf, geometry2等
- 開発ツール: clang-format, pytest等

### ビルド最適化

- ccacheの使用
- マルチステージビルド（将来的に検討可能）

## トラブルシューティング

### ビルドが遅い

ccacheが有効になっているか確認：

```bash
docker build --build-arg CCACHE_DIR=/ccache ...
```

### テストがタイムアウト

シナリオテストのタイムアウト設定を確認：

```bash
# test.xmlの timeout 属性を確認
grep timeout crane_planner_plugins/test/scenario_test/*.test.py
```

### 依存関係エラー

rosdepが最新か確認：

```bash
rosdep update
rosdep install --from-paths src --ignore-src -y
```

## 関連リンク

- [Docker環境全体の説明](../README.md)
- [シミュレーション開発環境](../sim/README.md)
- [シナリオテスト詳細](../../crane_planner_plugins/test/scenario_test/README.md)
- [CI設定](.github/workflows/ros-ci.yaml)
