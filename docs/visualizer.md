# 可視化システム: VisualizerMessageBuilderとCraneVisualizerBuffer

このドキュメントでは、Craneプロジェクトで使用される可視化システムの主要コンポーネントである`VisualizerMessageBuilder`と`CraneVisualizerBuffer`の使い方について説明します。これらを使って、ロボットの動き、ボールの位置、戦略の
判断などをリアルタイムで視覚的に表現できます。

Craneの可視化システムは、SVG（Scalable Vector Graphics）ベースの描画機能を提供します。この仕組みにより、フィールド上の様々な要素（ロボット、ボール、軌道、戦略情報など）を直感的に表示できます。

主要なコンポーネント：

- VisualizerMessageBuilder: 個々の描画要素（円、線、テキストなど）を生成するためのビルダーパターンを実装したクラス
- CraneVisualizerBuffer: 生成された描画要素をまとめてパブリッシュするためのバッファ機能を提供するクラス（更新トピック SvgUpdates を発行）

## 使用の流れ

基本的な使用の流れは以下の通りです：

1. VisualizerMessageBuilder のインスタンスを作成
2. CraneVisualizerBuffer を初期化（通常はノード起動時に一度だけ）
3. 各種ビルダー（Circle、Line、Text など）で描画要素を生成
4. 生成した描画要素を flush して更新に追加
5. バッファの内容を publish して表示

## 初期化

### VisualizerMessageBuilder の作成

通常、クラスのメンバ変数として VisualizerMessageBuilder のインスタンスを保持します：

```cpp
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>

class MyNode : public rclcpp::Node
{
private:
  crane::VisualizerMessageBuilder::SharedPtr visualizer;

public:
  MyNode()
  : Node("my_node")
  {
    // レイヤー名を指定してビルダーを作成
    visualizer = std::make_shared<crane::VisualizerMessageBuilder>("my_layer");

    // CraneVisualizerBufferを初期化
    crane::CraneVisualizerBuffer::activate(*this);
  }
};
"my_layer" は描画要素のグループ名です。

### CraneVisualizerBuffer の初期化

CraneVisualizerBuffer はシングルトンパターンで、一度初期化するとプログラム全体で使用できます：
// デフォルトのトピック名（/visualizer_svgs, SvgUpdates）で初期化
crane::CraneVisualizerBuffer::activate(*this);

// カスタムトピック名で初期化
crane::CraneVisualizerBuffer::activate(*this, "/my_custom_visualizer_topic");
## 描画要素の作成（SVGプリミティブ）

VisualizerMessageBuilder は、以下の様々な描画ビルダーを提供します。各ビルダーはメソッドチェーンで流暢に扱えます。

- 円（Circle）
- 線（Line）
- テキスト（Text）
- 折れ線（Polyline）
- 多角形（Polygon）
- 矩形（Rectangle）
- パス（Path）

例（円）：
visualizer->circle()
  .center(Point(1.0, 2.0))
  .radius(0.2)
  .fill("blue", 0.5)
  .stroke("black")
  .strokeWidth(2.0)
  .build();
他のビルダーも同様です（Line, Text, Polyline, Polygon, Rect, Path）。

## 描画要素のパブリッシュ

作成した描画要素をパブリッシュするには、以下の手順が必要です：
// 描画要素を更新メッセージに変換してバッファに追加（デフォルトは replace）
visualizer->flush();

// バッファの内容をパブリッシュ
crane::CraneVisualizerBuffer::publish();
通常、この処理はコールバック関数の最後で行います：
void onTimer()
{
  // ... 描画要素の作成 ...

  // 描画要素をフラッシュしてパブリッシュ
  visualizer->flush();
  crane::CraneVisualizerBuffer::publish();
}
発行されるメッセージは次の構造です：

- 型: crane_visualization_interfaces/msg/SvgUpdates
    - std_msgs/Header header（stamp を設定）
    - uint32 epoch（ノード再起動や状態初期化時にインクリメント推奨）
    - uint32 seq（エポック内で単調増加）
    - SvgLayerUpdate[] updates
    - `string layer`
    - `string operation`（`replace` | `append` | `clear`）
    - `string[] svg_primitives`

CraneVisualizerBuffer::publish() は header.stamp/epoch/seq を自動で設定します。必要に応じて CraneVisualizerBuffer::setEpoch() を呼び出してください。
// 起動時や新しいログ区切り時にエポックを切り替える
crane::CraneVisualizerBuffer::setEpoch(1);
## レイヤー操作（replace/append/clear）

デフォルトでは flush() はそのレイヤーに対して operation="replace" を発行し、レイヤーの内容を完全に置き換えます。用途に応じて操作種別を切り替えられます。
// レイヤー全体の置換（デフォルト）
visualizer->asReplace();
// 既存の内容に追記（軌跡などの長尺データ向け）
visualizer->asAppend();
// レイヤーの全要素を削除
visualizer->asClear();

// 例：スコア表示を完全置換
visualizer->asReplace();
visualizer->text().position(Point(-4.5, -6.5)).text("Score: 1-0").fontSize(100).build();
visualizer->flush();

// 例：軌跡に追記（1点）
visualizer->asAppend();
visualizer->circle().center(ball_pos).radius(0.003).fill("red").build();
visualizer->flush();

// 例：一時マーカーをクリア
visualizer->asClear().flush();

crane::CraneVisualizerBuffer::publish();
注意：append はベースが無い時（古い時刻へシークした場合など）に無視されることがあります。重要な動的表示は replace を基本とし、軌跡などのみ append を併用してください。

## レイヤーの消去（バッファ vs 表示）

「表示の消去」と「送信前バッファの消去」は異なります。
// 送信済みの表示を消去したい場合（受信側にクリア命令を出す）
visualizer->asClear().flush();
crane::CraneVisualizerBuffer::publish();

// 送信前のローカル更新バッファからレイヤーを取り除きたい場合
crane::CraneVisualizerBuffer::clear("my_layer"); // 未送信の updates から該当レイヤーを削除
crane::CraneVisualizerBuffer::clear();           // 未送信の全 updates を破棄
## 実用的な例

### ロボットの状態を可視化する例

```cpp
void visualizeRobotState(const RobotInfo & robot)
{
  // ロボットの位置を円で表示
  visualizer->circle()
    .center(robot.pose.pos)
    .radius(0.09)
    .fill("blue", 0.5)
    .stroke("black")
    .strokeWidth(1.0)
    .build();

  // ロボットの向きを矢印で表示
  Point arrow_end = robot.pose.pos + getNormVec(robot.pose.theta) * 0.15;
  visualizer->line()
    .start(robot.pose.pos)
    .end(arrow_end)
    .stroke("white")
    .strokeWidth(2.0)
    .build();

  // ロボットIDを表示
  visualizer->text()
    .position(robot.pose.pos + Vector2(0, -0.15))
    .text(std::to_string(robot.id))
    .fontSize(30)
    .fill("white")
    .textAnchor("middle")
    .build();
}
```

### 軌道予測を可視化する例

```cpp
void visualizeTrajectory(const std::vector<Point> & trajectory_points)
{
  // 予測軌道を折れ線で表示
  auto polyline_builder = visualizer->polyline();

  for (const auto & point : trajectory_points) {
    polyline_builder.addPoint(point);
  }

  polyline_builder.stroke("green")
    .strokeWidth(2.0)
    .build();

  // 終点を円で強調
  if (!trajectory_points.empty()) {
    visualizer->circle()
      .center(trajectory_points.back())
      .radius(0.05)
      .fill("red")
      .build();
  }
}
```

軌跡を随時追加したい場合は append を利用します（適宜 clear でリセット）。

```cpp
// 新たな軌跡点を1つだけ追加
visualizer->asAppend();
visualizer->circle().center(next_point).radius(0.003).fill("cyan").build();
visualizer->flush();
crane::CraneVisualizerBuffer::publish();
```

### ヒートマップの描画例

```cpp
void visualizeHeatmap(const std::vector<std::vector<double>> & data,
                     double x_min, double y_min, double cell_size)
{
  for (size_t i = 0; i < data.size(); ++i) {
    for (size_t j = 0; j < data[i].size(); ++j) {
      double value = std::clamp(data[i][j], 0.0, 1.0);
      int r = static_cast<int>(255 * value);
      int b = static_cast<int>(255 * (1.0 - value));
      std::string color = "rgb(" + std::to_string(r) + ",0," + std::to_string(b) + ")";

      double x = x_min + j * cell_size;
      double y = y_min + i * cell_size;

      visualizer->rect()
        .top_left(Point(x, y))
        .size(Point(cell_size, cell_size))
        .fill(color, 0.7)
        .build();
    }
  }
}
```

## 注意点（パフォーマンスと時間シーク）

- Foxgloveパネル側は /aggregated_svgs（スナップショット: SvgSnapshot）と /visualizer_svgs（更新: SvgUpdates）を合成して任意時刻を復元します。スナップショットが無い時刻でも、replace/clear は表示可能です（append はベース不在時に無視されることがあります）。
- 過剰な描画要素の生成はパフォーマンスに影響するため、必要最小限の描画にとどめることを推奨します。
- SVGの色は名前（"red", "blue"）や RGB 形式（"rgb(255,0,0)"）で指定できます。
- 動的レイヤー（ロボット・ボール・ラベルなど）は原則 replace を推奨、長尺レイヤー（軌跡等）のみ append を併用し、適宜 clear でリセットしてください。

## トラブルシューティング

### 描画要素が表示されない

- flush() と publish() が呼び出されていることを確認してください。
- レイヤー名が正しく設定されていることを確認してください。
- 座標値が適切な範囲内にあることを確認してください。
- append のみを送っている場合、ベースが無くて表示されないことがあります（replace を先に送る／スナップショット間隔を短くする）。

### 描画要素が正しく表示されない

- 座標系が正しいか確認してください（メートル単位で指定されているか）。
- 色や線の太さなどのスタイルパラメータが適切か確認してください。
- 複雑な描画要素の場合、パスの定義が正しいか確認してください。
- 更新トピックに誤って旧形式（svg_primitive_arrays）を送っていないか確認してください（現行ラッパーは SvgUpdates を発行します）。

## まとめ

VisualizerMessageBuilder と CraneVisualizerBuffer を使うことで、ロボットサッカーの試合状況を視覚的に分かりやすく表示できます。適切な可視化は、デバッグ、分析、戦略の開発において非常に有用です。

効果的な可視化のポイント：

- 必要な情報を明確に表示する
- 色や形状で情報を区別する
- 複雑すぎる表示は避ける
- 定期的に更新する情報と静的な情報を適切に使い分ける
