# Docker環境

- [開発・シミュレーション・実機](dev/README.md)
- [シナリオテスト用コンテナ](scenario/README.md)
- [TIGERs対戦テスト](match-vs-tigers/README.md)

起動前に[ネットワーク隔離と実機への切替](../docs/network.md)を確認してください。

ビルドの構成は [Dockerfile](Dockerfile)、共有設定は [config/](config/)、起動の引数は [docker-dev.sh](../scripts/docker-dev.sh) が正本です。
