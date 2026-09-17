# crane_game_analyzer

## 役割

SSL-VisionおよびSSL-Refereeデータから戦術的判断に必要な情報をリアルタイムで抽出・評価し、ヒステリシス処理によって安定した戦術判断指標を上位層へ提供するパッケージです。

## 固有の制約

- 状況判定の急激なチャタリング（頻繁な切替）を防ぐため、`SelectionHysteresis`などの保持期間制御および改善率判定を適用すること。

## トピックガイド

- [パス連携（PassTargetMetric・ヒステリシス評価）](../docs/pass.md)

## リンク

- ソースディレクトリ: [crane_game_analyzer](https://github.com/ibis-ssl/crane/tree/develop/crane_game_analyzer)
