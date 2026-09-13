# crane_visualization_interfaces

## 役割

SVGベースのリアルタイム可視化用ROS 2メッセージ定義と、描画構築・バッファリング用ラッパー（`VisualizerMessageBuilder`, `CraneVisualizerBuffer`）を提供するパッケージです。

## 固有の制約

- 描画入力は常にフィールド座標系（m）で渡し、SVG変換（mm・Y軸反転）は内部で自動実行されること。
- スナップショット（`SvgSnapshot`、5秒周期）と差分更新（`SvgUpdates`）による階層描画モデルに従うこと。

## トピックガイド

- [可視化の設計契約](../docs/visualizer.md)

## リンク

- ソースディレクトリ: [crane_visualization_interfaces](https://github.com/ibis-ssl/crane/tree/develop/crane_visualization_interfaces)
- メッセージ定義: [msg](https://github.com/ibis-ssl/crane/tree/develop/crane_visualization_interfaces/msg)
- ラッパーヘッダー: [crane_visualizer_wrapper.hpp](https://github.com/ibis-ssl/crane/blob/develop/crane_visualization_interfaces/include/crane_visualization_interfaces/crane_visualizer_wrapper.hpp)
