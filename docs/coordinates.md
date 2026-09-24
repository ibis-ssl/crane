# 座標系仕様

## 目的

Craneシステム全体における位置・姿勢・速度・表示の表現を統一するための座標系および単位規約を定めます。

## 座標系および単位契約

### 1. フィールド座標系（グローバル座標系）

SSL-Visionおよび内部世界モデルで使用される共通座標系です。

- **原点**: フィールド中央（Center Circle中心）
- **単位**: メートル [m]
- **X軸**: フィールド長辺方向（ゴール間方向）。自チームゴールが負、相手チームゴールが正。
- **Y軸**: フィールド短辺方向（サイドライン間方向）。
- **Z軸**: 地面に垂直な方向（鉛直上向きが正）。

### 2. ロボット座標系（ローカル座標系）

各ロボットの中心を原点とする機体固定座標系です。

- **原点**: ロボット中心
- **単位**: メートル [m]
- **X軸**: ロボット前方（キッカー開口部方向）
- **Y軸**: ロボット左方
- **Z軸**: ロボット上方

### 3. 角度・回転規約

- **単位**: ラジアン [rad]
- **範囲**: $[-\pi, \pi]$
- **基準（0 rad）**: フィールド座標系X軸正方向
- **回転方向**: 反時計回り（CCW）が正（右手系）

### 4. 可視化（SVG）座標系

外部表示ツール（Foxglove、Webビューア等）向けのSVG描画における規約です。

- **単位**: ミリメートル [mm]（フィールド座標系 [m] の値を1000倍して変換）
- **軸反転**: SVG規格に従い、Y軸は下向きが正（内部ラッパー `VisualizerMessageBuilder` で自動反転処理）

## 実装リファレンス

- 幾何学演算・角度正規化: [geometry_operations.hpp](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_geometry/include/crane_geometry/geometry_operations.hpp)
- 可視化座標変換: [crane_visualizer_wrapper.hpp](https://github.com/ibis-ssl/crane/blob/develop/crane_visualization_interfaces/include/crane_visualization_interfaces/crane_visualizer_wrapper.hpp)
- ロボット運動モデル: [robot_info.hpp](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_physics/include/crane_physics/robot_info.hpp)
- 関連ドキュメント: [可視化の設計契約](./visualizer.md) | [局所経路計画](./rvo2_local_planner.md)
