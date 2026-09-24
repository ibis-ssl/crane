# Docker開発環境

シミュレーションと実機で共用する。リポジトリルートから起動スクリプトを使う。

```bash
./scripts/docker-dev.sh                         # ER-Force
./scripts/docker-dev.sh --minimal -d            # シミュレータ開発用の構成
./scripts/docker-dev.sh real                    # 実機
./scripts/docker-dev.sh down
```

## 運用上の注意

- simではマルチキャストをループバックへ限定し、物理インターフェースへの送出をiptablesで遮断する。ルート設定だけでは直接送出するツールを防げず、過去にアクセスポイントが過負荷で停止した。起動スクリプトの隔離確認を省略しない。realでは隔離を解除してから起動する。設定手順は[Docker運用](../../docs/docker.md)を参照する。
- Robot Manager は `web-debugger` (8090) に内包している（`/robot_manager/`、API は `/api/robot-manager/*`）。以前は Orion_CM4 側が管理する別サービス (8092) だったが、共通テーマをコピーして持っていてドリフトの温床だったため取り込んだ。Orion_CM4 には Pi 側 API の契約文書だけが残る。
- 実機への HTTP ポーリングは、環境変数 `ROBOT_MANAGER_ENABLED`（既定 `0`）で止める。`0` のときサーバは `/robots` 系をネットワークに一切触れずに 503 で返し、クライアントもポーリングを開始しない。したがって sim では 192.168.20.0/24 へパケットが 1 つも出ない。実機で使うには `./scripts/docker-dev.sh --robot-manager` を指定する（`--minimal` と併用しても効く）。
- Ctrl+Cだけでは `ssl-log-recorder` が動き続ける。`down` まで実行する。
- ER-Force構成には実機CM4に相当する `cm4-sim` が含まれ、位置制御ループを閉じる。ホスト側のCraneは `planner:=visibility_graph` で起動する（ポート・アドレスやフィードバック受信方式は自動設定される）。詳細は[ネットワーク](../../docs/network.md)を参照する。

Webデバッガーの入口は <http://localhost:8090/>。Viewer・Annotation・Robot Manager はすべてこの同一オリジンで配信される（ロボット単位のテレメトリとテストは Viewer のサイドバータブ `?robot=<id>&tab=telemetry|test` に統合済み）。サービス・ポート・profile・設定値は[Compose](https://github.com/ibis-ssl/crane/blob/develop/docker/dev/docker-compose.yaml)、引数と起動処理は[docker-dev.sh](https://github.com/ibis-ssl/crane/blob/develop/scripts/docker-dev.sh)を参照する。
