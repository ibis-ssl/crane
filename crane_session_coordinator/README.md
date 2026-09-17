# crane_session_coordinator

## 役割

SSL-Referee指示および試合状況（PlaySituation）に基づき、利用可能なロボットへの戦術セッション（`crane_sessions`）の動的割り当てと試合進行を統括する最上位制御パッケージです。

## 固有の制約

- 設定駆動アーキテクチャを採用し、すべての試合状況設定は `config/unified_session_config.yaml`（`events` → `situations` → `sessions`）で管理すること。

## 関連ガイド

- [競技ルールへの対応](../docs/rule.md)
- [Crane ガイド（アーキテクチャ）](../docs/index.md)

## リンク

- ソースディレクトリ: [crane_session_coordinator](https://github.com/ibis-ssl/crane/tree/develop/crane_session_coordinator)
- 設定ファイル: [unified_session_config.yaml](https://github.com/ibis-ssl/crane/blob/develop/crane_session_coordinator/config/unified_session_config.yaml)
- テスト: [test](https://github.com/ibis-ssl/crane/tree/develop/crane_session_coordinator/test)
