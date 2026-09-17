# crane_local_planner

## 役割

上位スキルからの目標位置・速度を受け取り、RVO2アルゴリズム等を中核として動的障害物回避と動力学制約を満たす実現可能なロボット制御コマンドを生成するパッケージです。

## 固有の制約

- **実機使用制限**: 実機で位置目標モード（`POSITION_TARGET_MODE`）を使用する場合は、ロボット側に位置制御ループを担う CM4（[Orion_CM4](https://github.com/ibis-ssl/Orion_CM4)）が必要です。詳細は [CM4・cm4-simでの位置制御](../docs/cm4_position_control.md) を参照してください。CM4 による位置制御を行わない実機構成では、速度目標モード（mode 3）を出力する既定の `rvo2` を選択してください。
- ルール上の制約として、ペナルティエリア回避、ボール回避（セットプレー時）、ボールプレイスメントエリア回避を厳格に順守すること。

## トピックガイド

- [RVO2 Local Planner](../docs/rvo2_local_planner.md)

## リンク

- ソースディレクトリ: [crane_local_planner](https://github.com/ibis-ssl/crane/tree/develop/crane_local_planner)
- テスト: [test](https://github.com/ibis-ssl/crane/tree/develop/crane_local_planner/test)
