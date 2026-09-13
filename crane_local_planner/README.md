# crane_local_planner

## 役割

上位スキルからの目標位置・速度を受け取り、RVO2アルゴリズム等を中核として動的障害物回避と動力学制約を満たす実現可能なロボット制御コマンドを生成するパッケージです。

## 固有の制約

- **実機使用制限**: 現行の実機ファームウェアは位置目標モード（`POSITION_TARGET_MODE`）に対応していないため、ファームウェア側の対応が完了するまで `planner:=visibility_graph` はシミュレーション環境限定とし、実機では選択しないこと（既定値の `rvo2` を使用すること）。
- ルール上の制約として、ペナルティエリア回避、ボール回避（セットプレー時）、ボールプレイスメントエリア回避を厳格に順守すること。

## トピックガイド

- [RVO2 Local Planner](../docs/rvo2_local_planner.md)

## リンク

- ソースディレクトリ: [crane_local_planner](https://github.com/ibis-ssl/crane/tree/develop/crane_local_planner)
- テスト: [test](https://github.com/ibis-ssl/crane/tree/develop/crane_local_planner/test)
