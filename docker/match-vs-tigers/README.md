# TIGERs Sumatraとの対戦

ER-Force上でCraneとSumatraを対戦させ、スコア・イベントを保存する。
Docker Composeと利用可能なCraneイメージが必要。リポジトリルートから実行する。

```bash
CRANE_TAG=<利用可能なタグ> ./scripts/match-vs-tigers/run_local.sh
```

このスクリプトは開始時に既存コンテナ・ボリュームを停止・削除し、`docker/match-vs-tigers/results/` を消去する。前回の結果が必要なら先に退避する。

## ネットワークと結果

- 各サービスはhost networkを使う。起動時のマルチキャスト隔離確認を省略しない。ループバックへのルーティングと物理インターフェースへの遮断が必要。[Docker運用](../../docs/docker.md)を参照し、実機運用前には隔離を解除する。
- 試合結果は `docker/match-vs-tigers/results/match_result.txt`。通信ログとrosbagは同対戦ディレクトリ配下の `ssl-logs/` と `crane-rosbag/` に保存する。
- 結果ファイルがない場合はコントローラと各サービスのログを確認する。スクリプト終了だけで試合成立を判断しない。
- 中断時・コンテナ維持を選んだ場合は、ログ取得後に明示的に停止する。

```bash
docker compose -f docker/match-vs-tigers/docker-compose.yaml logs
docker compose -f docker/match-vs-tigers/docker-compose.yaml down
```

試合時間・使用イメージ・設定は[Compose](https://github.com/ibis-ssl/crane/blob/develop/docker/match-vs-tigers/docker-compose.yaml)、開始・終了条件は[試合コントローラ](https://github.com/ibis-ssl/crane/blob/develop/docker/match-vs-tigers/scripts/match_controller_pb.py)、ローカル後処理は[実行スクリプト](https://github.com/ibis-ssl/crane/blob/develop/scripts/match-vs-tigers/run_local.sh)を参照する。
過去の限定的な検証記録は[TESTING_NOTES](TESTING_NOTES.md)に残す。
