# TIGERs Sumatra AIとの対戦環境

TIGERs Mannheimの強豪AI「Sumatra」と対戦し、結果をテキストサマリーで出力するCI環境です。

## 概要

- **チーム構成**: crane (Yellow) vs TIGERs Sumatra (Blue)
- **試合時間**: キックオフから90秒（前半45秒 + 後半45秒）
- **出力**: テキストサマリー（スコア、勝敗、主要イベント）
- **ネットワーク**: Dockerブリッジネットワーク（ホストモード不使用）
- **GUI**: headlessモードのみ

## ディレクトリ構成

```text
docker/match_vs_tigers/
├── README.md                         # このファイル
├── docker-compose.yaml               # メイン構成
├── config/
│   ├── ssl-game-controller-match.yaml  # 90秒試合の設定
│   └── tigers-config.yaml            # TIGERs Sumatra設定
├── scripts/
│   └── match_controller.py           # 試合制御・結果出力
└── results/                          # 結果出力ディレクトリ（.gitignore）
    └── match_result.txt              # 試合結果サマリー
```

## ローカルでの実行

### 前提条件

- Docker & Docker Compose がインストール済み
- craneのDockerイメージが利用可能（`ghcr.io/ibis-ssl/crane:latest`）

### 実行方法

```bash
# プロジェクトルートから実行
./scripts/match_vs_tigers/run_local.sh
```

または、直接Docker Composeを使用：

```bash
cd docker/match_vs_tigers
docker compose up
```

### 特定のcraneイメージタグを使用

```bash
CRANE_TAG=develop ./scripts/match_vs_tigers/run_local.sh
```

## GitHub Actionsでの実行

### 手動実行

1. GitHub Actionsタブを開く
2. "TIGERs対戦CI" ワークフローを選択
3. "Run workflow" ボタンをクリック

### 自動実行

- **スケジュール**: 毎週月曜日 9:00 (JST) に自動実行
- **トリガー**: `.github/workflows/match_vs_tigers.yaml` で設定変更可能

## サービス構成

### Docker Composeサービス

1. **grsim** - シミュレータ
   - イメージ: `ghcr.io/ssl-roots/docker_images/grsim:main`
   - headlessモード

2. **ssl-game-controller** - 試合制御
   - イメージ: `ghcr.io/robocup-ssl/ssl-game-controller:3.17.2`
   - ポート: 8081 (HTTP API)

3. **autoref-tigers** - 自動審判
   - イメージ: `tigersmannheim/auto-referee:1.2.0`

4. **crane** - Yellow Team (ibis)
   - イメージ: `ghcr.io/ibis-ssl/crane:${CRANE_TAG}`

5. **tigers** - Blue Team (TIGERs Sumatra)
   - イメージ: `tigersmannheim/sumatra:2024.2.0`

6. **match-controller** - 試合制御・結果出力
   - Python 3.12で実行
   - 結果を `results/match_result.txt` に出力

### ネットワーク構成

- カスタムブリッジネットワーク `match` を使用
- マルチキャスト通信は同一ネットワーク内のコンテナ間で有効
- ホスト環境には影響しない

## 試合設定

### 試合時間

- 前半: 45秒
- ハーフタイム: 2秒
- 後半: 45秒
- 合計: 約90秒

### 自動承認設定

- `auto-approve-goals: true` - ゴールを自動承認
- `continue-from-halt: true` - ハーフタイム後に自動継続

詳細: `config/ssl-game-controller-match.yaml`

## 出力サンプル

```text
=====================================
        TIGERs対戦結果サマリー
=====================================

【スコア】
  ibis (Yellow): 2
  TIGERs Mannheim (Blue): 1

【勝敗】
  CRANE WIN (ibis)

【試合時間】
  90.3秒

【主要イベント】
  - [23.5s] GOAL by Yellow
  - [45.2s] GOAL by Blue
  - [78.1s] GOAL by Yellow

=====================================
```

## トラブルシューティング

### サービスが起動しない

```bash
# ログを確認
docker compose -f docker/match_vs_tigers/docker-compose.yaml logs

# 特定のサービスのログ
docker compose -f docker/match_vs_tigers/docker-compose.yaml logs crane
```

### マルチキャスト通信の問題

- Dockerブリッジネットワーク内ではマルチキャストが動作するはず
- ホストのファイアウォール設定を確認

### TIGERs Sumatraが起動しない

- `tigersmannheim/sumatra` イメージが利用可能か確認
- headlessモードの設定を確認: `config/tigers-config.yaml`

## 実装詳細

### match_controller.py

試合制御スクリプトは以下の機能を実装：

1. **HTTP API経由の試合監視**
   - ssl-game-controllerの `/api/state` エンドポイントを使用
   - スコア、ステージ変化をリアルタイムで追跡
   - POST_GAMEステージで自動終了

2. **試合開始シーケンス**
   - NORMALフェーズへの移行
   - キックオフコマンドの送信

3. **イベント記録**
   - ゴールイベント（Yellow/Blue）
   - ステージ変化
   - タイムスタンプ付きイベントログ

### 今後の改善点

1. **詳細ログ**
   - ボール位置、ロボット位置の記録
   - ファウル、カードの詳細
   - Protocol Buffersを使用したレフェリーメッセージの完全解析

2. **統計情報**
   - ボール保持率
   - パス成功率
   - シュート統計

## 参考資料

- [ssl-game-controller](https://github.com/RoboCup-SSL/ssl-game-controller)
- [TIGERs Mannheim Sumatra](https://github.com/TIGERs-Mannheim/Sumatra)
- [grSim](https://github.com/RoboCup-SSL/grSim)
