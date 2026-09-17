# grSim

Crane で利用可能な SSL 公式シミュレータ [ibis-ssl/grSim](https://github.com/ibis-ssl/grSim) のガイドです。

## 起動手順

Docker 開発環境からプロファイルを指定して起動します。

```bash
./scripts/docker-dev.sh --sim grsim
```

停止は `./scripts/docker-dev.sh down` を行います。

## Crane との連携

`grSim` は RobotControl protobuf 形式を受け付けるため、`packet_type:=ssl` を指定して起動します。

```bash
ros2 launch crane_bringup crane.launch.xml sim:=true packet_type:=ssl
```

送信形式・フィールド形状・Vision/Referee の接続先を合わせ、起動前に[ネットワーク隔離](network.md)が適用されていることを確認してください。

## フォーク版の変更点

オリジナル ([RoboCup-SSL/grSim](https://github.com/RoboCup-SSL/grSim)) から以下の変更を加えています：

- **ドリブル離脱**: ドリブル中にボールへ一定の外力がかかった場合にボールを離す挙動を追加。
- **ペナルティエリア情報**: 公式 Vision と同様に、ペナルティエリア寸法（オプショナルフィールド）を出力するように拡張。

## 実装・設定リファレンス

- [docker-compose.yaml](https://github.com/ibis-ssl/crane/blob/develop/docker/dev/docker-compose.yaml)（grsim サービス定義）
- [grsim_component.cpp](https://github.com/ibis-ssl/crane/blob/develop/consai_ros2/robocup_ssl_comm/src/grsim_component.cpp)
- [Docker開発環境](docker.md) / [ネットワーク設定](network.md)
