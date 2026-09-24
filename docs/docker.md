# Docker環境

Docker Engine と Compose V2 を使用します。導入は [Docker公式手順](https://docs.docker.com/engine/install/ubuntu/)、起動・停止は [開発環境README](https://github.com/ibis-ssl/crane/blob/develop/docker/dev/README.md)を参照してください。

host network のシミュレーションでは、起動前のマルチキャスト隔離が必要です。実機へ切り替える際は解除が必要になるため、[ネットワーク設定](network.md)を確認してください。

## 用途別の入口

- [開発・シミュレーション・実機](https://github.com/ibis-ssl/crane/blob/develop/docker/dev/README.md)
- [シナリオテスト](https://github.com/ibis-ssl/crane/blob/develop/scenario_test/README.md)
- [TIGERs対戦テスト](https://github.com/ibis-ssl/crane/blob/develop/docker/match-vs-tigers/README.md)

Docker側のツール群を起動した後、ホスト側のCraneは[環境構築・起動](setup.md#起動)に従って起動します。
