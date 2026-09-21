# 開発・テスト

初回の取得・ビルドは[環境構築](setup.md)を参照してください。

## 変更後の確認

以下は ROS ワークスペースルートで実行します。`crane_physics` は変更したパッケージに置き換えてください。共有APIを変更した場合は利用側もビルド・テストします。

```bash
cd ~/ibis_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to crane_physics
source install/local_setup.bash
colcon test --packages-select crane_physics --event-handlers console_cohesion+
colcon test-result --verbose
```

動作シナリオの検証は [シナリオテスト手順](https://github.com/ibis-ssl/crane/blob/develop/scenario_test/README.md)に従います。実行対象・補助コマンドは [Makefile](https://github.com/ibis-ssl/crane/blob/develop/Makefile)、CIの検証範囲は [workflows](https://github.com/ibis-ssl/crane/tree/develop/.github/workflows) が正本です。

## コミット前

pre-commit を導入し、リポジトリルートで実行します。

```bash
pipx install pre-commit
pre-commit install
pre-commit run --files <変更したファイル>
git diff --check
git diff --cached
```

フックはファイルを修正することがあるため、実行後の差分も確認します。全体検査は `pre-commit run --all-files`。チェック項目や書式は [.pre-commit-config.yaml](https://github.com/ibis-ssl/crane/blob/develop/.pre-commit-config.yaml) を参照してください。

## 調査の入口

- ROSノードとトピックの接続: `ros2 run rqt_graph rqt_graph`
- 起動後の異常: [診断](diagnostics.md)、[ネットワーク](network.md)
- ログ解析: [crane_mcap_tools](https://github.com/ibis-ssl/crane/tree/develop/crane_mcap_tools)、[SSL公式ツール](https://github.com/RoboCup-SSL/ssl-go-tools)
- 指令パケットの切り分け: [crane_packet_forge](https://github.com/ibis-ssl/crane/tree/develop/crane_packet_forge)（craneを経由せず任意のパケットを組み立てて送る。CLIとGUI）
