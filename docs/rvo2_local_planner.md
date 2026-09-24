# 局所経路計画

## 目的

上位スキルの目標を、他ロボット・ボール・ペナルティエリア・プレイスメント領域・フィールド境界を考慮した実行可能なコマンドへ変換します。

## プランナー

- `rvo2`（既定）: RVO2で他ロボットとの衝突を回避し、速度・加減速制約を適用する。実機とシミュレーションで使用する。
- `visibility_graph`: 相対速度から予測した障害物を可視グラフで回避し、先読み位置指令（`POSITION_TARGET_MODE`）を生成する。

実機で位置目標モード（`POSITION_TARGET_MODE` / ワイヤ mode 4）を運用する場合は、ロボット側に位置制御ループを実行する CM4（[Orion_CM4](https://github.com/ibis-ssl/Orion_CM4)）が必要です。詳細は [CM4・cm4-simでの位置制御](cm4_position_control.md) を参照してください。CM4 による位置制御を行わない実機環境では、速度目標モード（mode 3）を出力する既定の `rvo2` を選択します。

## 制約

- ペナルティエリア回避、ボール回避、プレイスメント領域回避を無効化・拡大するときは、競技ルールと実機挙動への影響を確認する。ペナルティエリアのグローバル回避マージンを安易に拡大しない。
- プランナーのパラメータ、障害物の扱い、再計画と速度制約は実装を正本とする。文書に既定値を転載しない。

## 実装リファレンス

- [LocalPlannerComponent](https://github.com/ibis-ssl/crane/blob/develop/crane_local_planner/include/crane_local_planner/local_planner.hpp)
- [RVO2Planner](https://github.com/ibis-ssl/crane/blob/develop/crane_local_planner/src/rvo2_planner.cpp)
- [VisibilityGraphPlanner](https://github.com/ibis-ssl/crane/blob/develop/crane_local_planner/src/visibility_graph_planner.cpp)
- [テスト](https://github.com/ibis-ssl/crane/tree/develop/crane_local_planner/test)
- [ルール制約](./rule.md) / [座標・単位](./coordinates.md) / [CM4・cm4-simでの位置制御](cm4_position_control.md)
