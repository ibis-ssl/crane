# シナリオテスト用Docker環境

実行手順・結果確認は[シナリオテスト](../../scenario_test/README.md)を参照する。

- [ローカル構成](https://github.com/ibis-ssl/crane/blob/develop/docker/scenario/docker-compose.local.yaml)は周辺サービスを起動し、Crane本体は実行スクリプトがホストで起動する。
- [イメージ構成](https://github.com/ibis-ssl/crane/blob/develop/docker/scenario/docker-compose.yaml)はCraneもコンテナで起動する。ビルドは `make scenario-test-build` を使う。
- どちらもhost networkを使うため、マルチキャスト隔離が必要。起動は隔離確認を含むMakefile経由とし、[Docker運用](../../docs/docker.md)の注意に従う。

イメージのビルド構成は[Dockerfile](https://github.com/ibis-ssl/crane/blob/develop/docker/Dockerfile)、CIの実行・成果物は[ワークフロー](https://github.com/ibis-ssl/crane/blob/develop/.github/workflows/scenario_test.yaml)を正本とする。
