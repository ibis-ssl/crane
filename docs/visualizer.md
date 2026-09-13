# 可視化の設計契約

`VisualizerMessageBuilder` はフィールド座標（m）の描画をレイヤー単位の SVG 更新へ変換します。API の全一覧は [`crane_visualizer_wrapper.hpp`](https://github.com/ibis-ssl/crane/blob/develop/crane_visualization_interfaces/include/crane_visualization_interfaces/crane_visualizer_wrapper.hpp) を参照してください。

- ノード初期化時に `CraneVisualizerBuffer::activate()` を 1 回呼び、描画周期では builder にプリミティブを追加して `flush()` する。最後に `CraneVisualizerBuffer::publish()` が更新を送信する。
- `asReplace()`、`asAppend()`、`asClear()` はレイヤーの更新操作を選ぶ。レイヤー名を機能単位で固定し、別機能のレイヤーを消去しない。
- 入力はフィールド座標系（m）。SVG への変換は実装が行う（倍率 1000、Y 軸反転）。軸の定義は [座標系仕様](./coordinates.md) を正本とする。
- `/visualizer_svgs` は各ノードの更新、`/aggregated_svgs` は集約ノードの完全スナップショット（5 秒周期）である。新しい購読者や再生処理は後者を利用する。

実装を変更したら、レイヤーの replace/append/clear と、可視化ノードを再起動した直後のスナップショットを確認します。

参照:

- [ラッパー実装](https://github.com/ibis-ssl/crane/tree/develop/crane_visualization_interfaces)
- [集約ノード](https://github.com/ibis-ssl/crane/blob/develop/crane_visualization_interfaces/src/visualization_aggregator.cpp)
- [メッセージ定義](https://github.com/ibis-ssl/crane/tree/develop/crane_visualization_interfaces/msg)
