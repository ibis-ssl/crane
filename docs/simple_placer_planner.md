# SimplePlacerPlanner

> **最終更新**: 2025年6月22日（JapanOpen2025後）  
> **関連パッケージ**: [crane_planner_plugins](./packages/crane_planner_plugins.md)  
> **現在の状況**: プラグインアーキテクチャに統合済み

`SimplePlacerPlanner`は、crane_planner_pluginsパッケージのプラグインシステムに統合されたエリア配置戦略プランナーです。

## 概要

このプランナーは、フィールドをいくつかのエリアに分割し、各ロボットを最も適切なエリアに割り当てることを目的としています。現在のプラグインアーキテクチャでは、`crane_session_controller`からの戦略指示に基づいて動的に選択・実行されます。

## プラグインアーキテクチャでの統合

### crane_planner_pluginsでの位置づけ

- **プラグインベース**: `PlannerBase`クラスを継承したプラグイン実装
- **動的ロード**: YAMLファイルによる戦略選択とパラメータ設定
- **セッション制御**: `crane_session_controller`による実行タイミング制御

### 主要機能

- **エリア分割**: フィールドをFW, MF, WG1, WG2などのエリアに分割
- **動的割り当て**: 敵ロボット配置に基づく最適エリア割り当て
- **再割り当てシステム**: クールダウン期間付きの再最適化
- **可視化統合**: crane_visualizerとの連携による戦術可視化

### メンバ変数

- `defense_areas`: `AreaWithInfo`構造体のベクター。各エリアの名前、自チームロボット数、敵チームロボット数、エリアのボックス情報を含みます。
- `assignment_map`: ロボットIDからエリア名へのマッピングを保持するunordered_map。
- `target_positions`: ロボットIDから目標位置へのマッピングを保持するunordered_map。
- `reassignment_cooldown`: ロボットIDから再割り当てクールダウンカウンターへのマッピングを保持するunordered_map。
- `position_threshold`: 移動が完了したと判断する距離しきい値。
- `cooldown_frames`: エリア再割り当ての待機フレーム数。

### メソッド

- `calculateRobotCommand`: ロボットコマンドを計算し、各ロボットを最適なエリアに移動させるためのコマンドを生成します。
- `getSelectedRobots`: ロボットを選択し、各ロボットに役割を割り当てます。

## 現在の実装状況

### crane_planner_pluginsでの統合

- **プラグイン登録**: プランナープラグインとして登録済み
- **設定駆動**: YAMLファイルによるエリア定義とパラメータ調整
- **実行制御**: SessionControllerによる試合状況に応じた選択的実行

### JapanOpen2025での運用実績

- **エリア戦術**: 相手チームの配置に応じた効果的なエリア占有
- **適応性**: 試合中の戦況変化への動的対応
- **可視化**: ゲーム分析での戦術理解支援

## 技術的詳細

このプランナーは、`PlannerBase`クラスを継承し、`calculateRobotCommand`メソッドと`getSelectedRobots`メソッドを実装します。ワールドモデルの情報に基づいて、各ロボットを最適なエリアに動的に割り当て、チーム全体の戦略的な配置を支援します。

### 関連システム

- **crane_geometry**: エリア境界の幾何学計算
- **crane_physics**: ロボット移動時間の計算
- **crane_local_planner**: 各ロボットの安全な経路計画

---

**詳細な実装**: [crane_planner_plugins](./packages/crane_planner_plugins.md)のプラグインシステムを参照
