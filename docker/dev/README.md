# Docker開発環境

シミュレーションと実機で共用する。リポジトリルートから起動スクリプトを使う。

```bash
./scripts/docker-dev.sh                         # ER-Force
./scripts/docker-dev.sh --sim grsim             # grSim
./scripts/docker-dev.sh --minimal -d            # シミュレータ開発用の構成
./scripts/docker-dev.sh real                    # 実機
./scripts/docker-dev.sh down
```

## 運用上の注意

- simではマルチキャストをループバックへ限定し、物理インターフェースへの送出をiptablesで遮断する。ルート設定だけでは直接送出するツールを防げず、過去にアクセスポイントが過負荷で停止した。起動スクリプトの隔離確認を省略しない。realでは隔離を解除してから起動する。設定手順は[Docker運用](../../docs/docker.md)を参照する。
- `robot-manager` は ROS 非依存で [Orion_CM4](https://github.com/ibis-ssl/Orion_CM4) 側が管理する実機管理UIである。実機へのHTTPポーリングを避けるためsimでは無効、realでは有効。simで必要な場合は `--robot-manager` を指定する。
- Ctrl+Cだけでは `ssl-log-recorder` が動き続ける。`down` まで実行する。
- ER-Force構成には実機CM4に相当する `cm4-sim` が含まれ、位置制御ループを閉じる。ホスト側のCraneは `planner:=visibility_graph` で起動する（ポート・アドレスやフィードバック受信方式は自動設定される）。詳細は[ネットワーク](../../docs/network.md)を参照する。

Webデバッガーの入口は <http://localhost:8090/>。サービス・ポート・profile・設定値は[Compose](https://github.com/ibis-ssl/crane/blob/develop/docker/dev/docker-compose.yaml)、引数と起動処理は[docker-dev.sh](https://github.com/ibis-ssl/crane/blob/develop/scripts/docker-dev.sh)を参照する。
