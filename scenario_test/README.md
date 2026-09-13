# シナリオテスト

シミュレータと自動審判を使い、pytestでCraneの動作を検証する。テスト名と合格条件は[各テスト](https://github.com/ibis-ssl/crane/tree/develop/scenario_test)を正本とする。

## 準備と実行

Docker Compose、Python 3.12のvenv、ビルド済みROS 2ワークスペースが必要。
セットアップはPython依存を取得し、protocがなければsudoで導入する。ログ取得・失敗動画の生成にはネットワークアクセス、Go、ffmpegも必要となる。

以下はリポジトリルートで実行する。

```bash
make scenario-test-setup
make scenario-test TEST=STOP_ROBOT_SPEED
make scenario-test PLANNER=visibility_graph TEST=VISIBILITY_OBSTACLE_AVOIDANCE
make scenario-test
```

既定のローカルモードはホストでCraneを起動する。コード変更後はワークスペースルートで対象パッケージを再ビルドし、環境を再読み込みしてから再実行する。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select <変更したパッケージ>
source install/local_setup.bash
```

イメージ内のCraneを検証する場合は、リポジトリルートで次を使う。

```bash
make scenario-test-build
USE_LOCAL=0 make scenario-test TEST=STOP_ROBOT_SPEED
```

タグなどの選択は `make help` と[Makefile](https://github.com/ibis-ssl/crane/blob/develop/Makefile)を参照する。

## ネットワークと終了確認

シミュレータはhost networkを使う。起動スクリプトがホストのマルチキャスト隔離を確認するため、sudoを要求する場合がある。ループバックへのルートだけでなく物理インターフェースへの遮断が必要。[Docker運用](../docs/docker.md)の隔離手順に従い、実機運用前には解除する。

通常終了時はCraneとコンテナを停止する。途中の環境エラーや中断時は残存プロセス・コンテナを確認する。手動のDocker起動・停止は `make scenario-test-docker-up` / `make scenario-test-docker-down` を使うが、ローカルモードのCraneは別途起動が必要。

## 結果の読み方

- 合否はpytestの結果で確認する。環境の起動失敗とシナリオのassert失敗を区別する。
- 通信ログはリポジトリルートの `*.log.gz`、ローカルCraneログは `/tmp/crane_local.log`。失敗時は最新の通信ログ1件から動画生成を試みるため、動画がないことだけでは成功と判断できない。
- CIの保存物・実行条件は[ワークフロー](https://github.com/ibis-ssl/crane/blob/develop/.github/workflows/scenario_test.yaml)、起動・後処理は[run_test.sh](https://github.com/ibis-ssl/crane/blob/develop/scripts/scenario_test/run_test.sh)を参照する。
- `make scenario-test-clean` は仮想環境、対象イメージ、取得ツール、ログ・動画を削除する。必要な記録は先に退避する。

## パス成功率の比較

シミュレータとCraneを起動した状態で、リポジトリルートから実行する。

```bash
source scenario_test_env/bin/activate
cd scenario_test
python measure_pass_rate.py --trials 20 --scenario buildup --out pass_rate_before.json
```

これはassertを行わないA/B比較用の計測。vision情報によるヒューリスティックで、判定は[pass_helpers.py](https://github.com/ibis-ssl/crane/blob/develop/scenario_test/pass_helpers.py)を参照する。bagからの詳細KPIには `crane_bag pass` を使う。
