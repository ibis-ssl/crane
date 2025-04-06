# 可視化システム: VisualizerMessageBuilderとCraneVisualizerBuffer

このドキュメントでは、Craneプロジェクトで使用される可視化システムの主要コンポーネントである`VisualizerMessageBuilder`と`CraneVisualizerBuffer`の使い方について説明します。これらのクラスを使用することで、ロボットの動き、ボールの位置、戦略の判断などをリアルタイムで視覚的に表現することができます。

## 概要

Craneの可視化システムは、SVG（Scalable Vector Graphics）ベースの描画機能を提供します。この仕組みにより、フィールド上の様々な要素（ロボット、ボール、軌道、戦略情報など）を直感的に表示できます。

主要なコンポーネント：

- **VisualizerMessageBuilder**: 個々の描画要素（円、線、テキストなど）を生成するためのビルダーパターンを実装したクラス
- **CraneVisualizerBuffer**: 生成された描画要素をまとめてパブリッシュするためのバッファ機能を提供するクラス

## 使用の流れ

基本的な使用の流れは以下の通りです：

1. `VisualizerMessageBuilder`のインスタンスを作成
2. `CraneVisualizerBuffer`を初期化（通常はノード起動時に一度だけ）
3. 様々なビルダー（Circle、Line、Textなど）を使って描画要素を生成
4. 生成した描画要素をフラッシュしてバッファに追加
5. バッファの内容をパブリッシュして表示

## 初期化

### VisualizerMessageBuilderの作成

通常、クラスのメンバ変数として`VisualizerMessageBuilder`のインスタンスを保持します：

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
```

`"my_layer"`は描画要素のグループ名で、複数のノードから描画要素が送信された場合に、どのノードからの描画かを識別するために使用されます。

### CraneVisualizerBufferの初期化

`CraneVisualizerBuffer`はシングルトンパターンで実装されており、一度初期化するとプログラム全体で使用できます：

```cpp
// デフォルトのトピック名（/visualizer_svgs）で初期化
crane::CraneVisualizerBuffer::activate(*this);

// カスタムトピック名で初期化
crane::CraneVisualizerBuffer::activate(*this, "/my_custom_visualizer_topic");
```

## 描画要素の作成

`VisualizerMessageBuilder`は、様々な種類の描画要素を生成するためのメソッドを提供しています。各ビルダーはメソッドチェーンパターンを採用しており、流暢なAPIを提供します。

### 円（Circle）の描画

```cpp
visualizer->circle()
  .center(Point(1.0, 2.0))     // 中心座標（メートル単位）
  .radius(0.2)                 // 半径（メートル単位）
  .fill("blue", 0.5)           // 塗りつぶしの色と透明度
  .stroke("black")             // 輪郭の色
  .strokeWidth(2.0)            // 輪郭の太さ
  .build();                    // 描画要素を構築
```

### 線（Line）の描画

```cpp
visualizer->line()
  .start(Point(0.0, 0.0))      // 開始点
  .end(Point(1.0, 1.0))        // 終了点
  .stroke("red")               // 線の色
  .strokeWidth(3.0)            // 線の太さ
  .build();                    // 描画要素を構築
```

### テキスト（Text）の描画

```cpp
visualizer->text()
  .position(Point(1.0, 1.0))   // テキストの位置
  .text("Hello, World!")       // 表示するテキスト
  .fontSize(50)                // フォントサイズ
  .fill("white")               // テキストの色
  .textAnchor("middle")        // テキストのアンカー位置（"start", "middle", "end"）
  .build();                    // 描画要素を構築
```

### 折れ線（Polyline）の描画

```cpp
visualizer->polyline()
  .addPoint(Point(0.0, 0.0))   // 点を追加
  .addPoint(Point(1.0, 1.0))   // 点を追加
  .addPoint(Point(2.0, 0.0))   // 点を追加
  .stroke("green")             // 線の色
  .strokeWidth(2.0)            // 線の太さ
  .build();                    // 描画要素を構築
```

### 多角形（Polygon）の描画

```cpp
visualizer->polygon()
  .addPoint(Point(0.0, 0.0))   // 点を追加
  .addPoint(Point(1.0, 1.0))   // 点を追加
  .addPoint(Point(2.0, 0.0))   // 点を追加
  .fill("yellow", 0.3)         // 塗りつぶしの色と透明度
  .stroke("black")             // 輪郭の色
  .strokeWidth(1.0)            // 輪郭の太さ
  .build();                    // 描画要素を構築
```

### 矩形（Rectangle）の描画

```cpp
visualizer->rect()
  .top_left(Point(0.0, 0.0))   // 左上の座標
  .size(Point(1.0, 0.5))       // 幅と高さ
  .fill("purple", 0.4)         // 塗りつぶしの色と透明度
  .stroke("white")             // 輪郭の色
  .strokeWidth(1.5)            // 輪郭の太さ
  .build();                    // 描画要素を構築
```

### パス（Path）の描画

パスはより複雑な形状を描画するために使用します：

```cpp
visualizer->path()
  .definition.moveTo(Point(0.0, 0.0))             // 開始点に移動
  .lineTo(Point(1.0, 1.0))                        // 直線を引く
  .cubicBezierTo(                                 // 3次ベジェ曲線
    Point(1.5, 1.0), Point(1.5, 0.0), Point(2.0, 0.0))
  .closePath()                                    // パスを閉じる
  .fill("orange", 0.2)                           // 塗りつぶしの色と透明度
  .stroke("red")                                  // 輪郭の色
  .strokeWidth(2.0)                               // 輪郭の太さ
  .build();                                       // 描画要素を構築
```

## 描画要素のパブリッシュ

作成した描画要素をパブリッシュするには、以下の手順が必要です：

```cpp
// 描画要素をバッファに送信
visualizer->flush();

// バッファの内容をパブリッシュ
crane::CraneVisualizerBuffer::publish();
```

通常、この処理はコールバック関数の最後で行います：

```cpp
void onTimer()
{
  // ... 描画要素の作成 ...

  // 描画要素をフラッシュしてパブリッシュ
  visualizer->flush();
  crane::CraneVisualizerBuffer::publish();
}
```

## レイヤーの消去

特定のレイヤーの描画要素を消去するには：

```cpp
// 特定のレイヤーを消去
crane::CraneVisualizerBuffer::clear("my_layer");

// すべてのレイヤーを消去
crane::CraneVisualizerBuffer::clear();
```

## 実用的な例

### ロボットの状態を可視化する例

```cpp
void visualizeRobotState(const RobotInfo & robot)
{
  // ロボットの位置を円で表示
  visualizer->circle()
    .center(robot.pose.pos)
    .radius(0.09)  // ロボットの半径
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

### ヒートマップの描画例

```cpp
void visualizeHeatmap(const std::vector<std::vector<double>> & data,
                     double x_min, double y_min, double cell_size)
{
  for (size_t i = 0; i < data.size(); ++i) {
    for (size_t j = 0; j < data[i].size(); ++j) {
      // データの値に基づいて色を計算（0.0〜1.0を青から赤へ）
      double value = std::clamp(data[i][j], 0.0, 1.0);
      int r = static_cast<int>(255 * value);
      int b = static_cast<int>(255 * (1.0 - value));
      std::string color = "rgb(" + std::to_string(r) + ",0," + std::to_string(b) + ")";

      // セルの左上座標を計算
      double x = x_min + j * cell_size;
      double y = y_min + i * cell_size;

      // セルを描画
      visualizer->rect()
        .top_left(Point(x, y))
        .size(Point(cell_size, cell_size))
        .fill(color, 0.7)
        .build();
    }
  }
}
```

## 応用テクニック

### アニメーション効果

時間変化のある描画を行うことで、簡単なアニメーション効果を実現できます：

```cpp
// 時間に基づいて円の大きさを変化させる
double animation_phase = std::fmod(now().seconds(), 1.0) * 2.0 * M_PI;
double radius = 0.1 + 0.05 * std::sin(animation_phase);

visualizer->circle()
  .center(target_pos)
  .radius(radius)
  .fill("yellow", 0.6)
  .build();
```

## 注意点

- 描画要素は毎フレーム更新されるため、一度描画したものは明示的に描画し直さない限り次のフレームでは消えてしまいます。
- 過剰な描画要素の生成はパフォーマンスに影響を与える可能性があるため、必要最小限の描画にとどめることをお勧めします。
- SVGは基本的にHTML/CSS風の指定方法を使用しているため、色は名前（"red", "blue"など）またはRGB形式（"rgb(255,0,0)"）で指定できます。

## トラブルシューティング

### 描画要素が表示されない

- `flush()`と`publish()`が呼び出されていることを確認してください。
- レイヤー名が正しく設定されていることを確認してください。
- 座標値が適切な範囲内にあることを確認してください。

### 描画要素が正しく表示されない

- 座標系が正しいか確認してください（メートル単位で指定されているか）。
- 色や線の太さなどのスタイルパラメータが適切か確認してください。
- 複雑な描画要素の場合、パスの定義が正しいか確認してください。

## まとめ

`VisualizerMessageBuilder`と`CraneVisualizerBuffer`を使うことで、ロボットサッカーの試合状況を視覚的に分かりやすく表示することができます。適切な可視化は、デバッグ、分析、戦略の開発において非常に役立つツールとなります。

効果的な可視化のために、以下のポイントに注意してください：

- 必要な情報を明確に表示する
- 色や形状で情報を区別する
- 複雑すぎる表示は避ける
- 定期的に更新する情報と静的な情報を適切に使い分ける

これらのポイントを押さえることで、より効果的なビジュアライゼーションを実現できます。
