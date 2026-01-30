# 実機環境（docker/real）

実機ロボットとの接続・動作確認用のDocker環境です。

## 概要

この環境では以下のサービスが起動します：

- **ssl-game-controller** - SSL Game Controller（試合管理）
- **ssl-vision-client** - Visionデータクライアント
- **ssl-status-board** - 試合状況表示ボード
- **voicevox** - 音声合成エンジン（CPU版）
- **aivis-speech** - AIVIS音声エンジン（CPU版）

## 使い方

### 起動

```bash
cd docker/real
docker compose up
```

### 各サービスへのアクセス

| サービス | URL | 説明 |
|---------|-----|------|
| Game Controller | <http://localhost:8081> | 試合制御UI |
| Vision Client | <http://localhost:8082> | Visionデータ確認 |
| Status Board | <http://localhost:8083> | 試合状況ダッシュボード |
| VOICEVOX | <http://localhost:50021> | 音声合成API |
| AIVIS Speech | <http://localhost:10101> | 音声合成API |

### crane（ホスト環境）の起動

```bash
# ROSワークスペースルートで実行
cd ~/workspace/ibis_ws_2  # ワークスペースルート
ros2 launch crane_bringup crane.launch.xml sim:=false speak:=true
```

**重要**: `sim:=false` で実機モードを有効化します。

## 実機環境の前提条件

### ハードウェア

- SSL-Vision システムが稼働中
- 実機ロボット（AI-03B基板搭載）が通信可能
- ネットワーク接続が確立済み

### ネットワーク設定

- Vision マルチキャストアドレス: `224.5.23.2:10006`
- Referee マルチキャストアドレス: `224.5.23.1:11003`

※ シミュレーション環境とポート番号が異なる点に注意

## sim環境との違い

| 項目 | sim環境 | real環境 |
|------|---------|----------|
| Vision ポート | 10020 | 10006 |
| シミュレータ | grSim必須 | 不要（実機Vision使用） |
| 自動審判 | アクティブモード | 無効 |
| crane起動オプション | `sim:=true` | `sim:=false` |

## 音声合成について

sim環境と同じ設定です。詳細は [sim/README.md](../sim/README.md) を参照してください。

## トラブルシューティング

### Visionデータが届かない

1. SSL-Visionが起動しているか確認
2. マルチキャストルーティングの設定を確認
3. ファイアウォールがマルチキャストを許可しているか確認

```bash
# マルチキャストパケットの確認
tcpdump -i any host 224.5.23.2
```

### ロボットと通信できない

1. ロボットの電源が入っているか確認
2. ネットワーク接続を確認（有線LAN推奨）
3. AI-03B基板の設定を確認

```bash
# craneからのコマンド送信ログを確認
ros2 topic echo /command
```

### Game Controllerに接続できない

- ポート11003が使用可能か確認
- 他のGame Controllerインスタンスが起動していないか確認

## 関連リンク

- [Docker環境全体の説明](../README.md)
- [シミュレーション環境](../sim/README.md)
- [実機ロボット仕様](../../docs/ai-03b-board.md)（存在する場合）
