# シミュレーション開発環境（docker/sim）

grSimやGame Controllerを含む、日常的な開発・デバッグ作業用のDocker環境です。

## 概要

この環境では以下のサービスが起動します：

- **ssl-game-controller** - SSL Game Controller（試合管理）
- **ssl-vision-client** - Vision データクライアント
- **ssl-status-board** - 試合状況表示ボード
- **autoref-tigers** - TIGERs自動審判（アクティブモード）
- **voicevox** - 音声合成エンジン（CPU版）
- **aivis-speech** - AIVIS音声エンジン（CPU版）

## 使い方

### 起動

```bash
cd docker/sim
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

### シミュレータの追加起動

このDocker Composeには含まれていませんが、以下のいずれかを別途起動してください：

#### grSim（推奨）

```bash
# ホスト環境でgrSimを起動
# - GUIが使える環境で直接起動
# - Docker版も利用可能（VNC経由）
```

#### ER-Force Simulator

```bash
docker run --rm --network host \
  ghcr.io/ibis-ssl/framework:simulatorcli \
  -e GEOMETRY=2023B -e REALISM=Friendly
```

### crane（ホスト環境）の起動

```bash
# ROSワークスペースルートで実行
cd ~/workspace/ibis_ws_2  # ワークスペースルート
ros2 launch crane_bringup crane.launch.xml sim:=true speak:=false
```

## 設定ファイル

- `ssl-vision-client-config.json` - Vision Clientの設定（ローカル固有）

Game Controllerの設定は実行時に自動生成されます。

## 音声合成について

### VOICEVOX

- デフォルトはCPU版を使用
- GPU版に変更する場合は `docker-compose.yaml` で以下を変更：

  ```yaml
  image: voicevox/voicevox_engine:nvidia-latest
  ```

### AIVIS Speech

- CPU版のみ提供

## トラブルシューティング

### ポート競合

既に使用中のポートがある場合は、他のプロセスを停止してください：

```bash
# ポート使用状況の確認
lsof -i :8081  # Game Controller
lsof -i :8082  # Vision Client
lsof -i :8083  # Status Board
```

### Vision接続エラー

- シミュレータ（grSim等）が起動しているか確認
- マルチキャストアドレスの設定を確認（デフォルト: 224.5.23.2:10020）

## 関連リンク

- [Docker環境全体の説明](../README.md)
- [実機環境](../real/README.md)
- [シナリオテスト環境](../scenario/README.md)
