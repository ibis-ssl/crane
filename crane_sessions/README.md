# crane_sessions

## 役割

攻撃、守備、セットプレー、ボール配置など、各種試合状況に対応したマルチロボット協調戦略（セッション）の実装パッケージです。

## 固有の制約

- 各セッションは `SessionBase` を継承し、`session_factory` の静的ファクトリ登録マップ（`PLANNER_ENTRY`）経由で生成されること。

## トピックガイド

- [オフェンス戦術](../docs/offense.md)
- [ディフェンス戦術](../docs/defense.md)
- [パス連携](../docs/pass.md)

## リンク

- ソースディレクトリ: [crane_sessions](https://github.com/ibis-ssl/crane/tree/develop/crane_sessions)
- ファクトリ実装: [session_factory.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_sessions/src/session_factory.cpp)
