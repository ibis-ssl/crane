# 試合チェックリスト

- [ ] Visionポート合ってる？
  - 10006
  - フィールドラインが出てればOK
- [ ] GCポート合ってる？
  - 10003
  - Visualizerの出力とステータスボードの一致を確認
  - ローカルGCを変化させて影響がない
- [ ] トラッカーポート合ってる？
  - ローカルGCからの出力
  - ロボット描画されてればOK
- [ ] ROS関連ノードを一度クリーンアップ
  - `ps aux | grep ros | grep -v grep | awk '{ print "kill -9 " $2 }' | sh`
- [ ] dockerの立ち上げ合ってる？
  - docker psで確認

---

## TIGERs対戦CI環境

TIGERs Mannheimの強豪AI「Sumatra」との自動対戦環境が利用可能です（PR #1109, #1110, #1113）。

### 概要

- **チーム構成**: crane (Yellow) vs TIGERs Sumatra (Blue)
- **試合時間**: キックオフから90秒（前半45秒 + 後半45秒）
- **出力**: テキストサマリー（スコア、勝敗、主要イベント）
- **ネットワーク**: Dockerブリッジネットワーク（ホストモード不使用）
- **GUI**: headlessモードのみ

### 実行方法

#### ローカル実行

```bash
# プロジェクトルートから
./scripts/match_vs_tigers/run_local.sh

# または特定のcraneイメージタグを指定
CRANE_TAG=develop ./scripts/match_vs_tigers/run_local.sh
```

#### GitHub Actions実行

1. GitHub Actionsタブを開く
2. "TIGERs対戦CI" ワークフローを選択
3. "Run workflow" ボタンをクリック

**自動実行**: 毎週月曜日 9:00 (JST) に自動実行

### サービス構成

- **grsim**: シミュレータ（headless）
- **ssl-game-controller**: 試合制御（HTTP API: ポート8081）
- **autoref-tigers**: 自動審判
- **crane**: Yellow Team（craneシステム）
- **tigers**: Blue Team（TIGERs Sumatra）
- **match-controller**: 試合制御・結果出力（`match_controller.py`）

### 出力例

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

詳細: `docker/match_vs_tigers/README.md`
