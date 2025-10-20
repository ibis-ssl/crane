# 可視化APIガイド（VisualizerMessageBuilder / CraneVisualizerBuffer）

> 最終更新: 2025年6月（JapanOpen2025運用後のAPIに対応）

このドキュメントでは、`crane_msg_wrappers/crane_visualizer_wrapper.hpp` が提供する最新の可視化APIと、`crane_visualization_aggregator` を使った描画パイプラインを解説します。

## コンポーネント概要

- **VisualizerMessageBuilder**  
  レイヤー単位でSVGプリミティブを生成するビルダークラス。フィールド座標（m）で渡した値を自動的にSVG座標（mm・Y軸反転）へ変換します。
- **CraneVisualizerBuffer**  
  各ビルダーから集めた描画更新を `/visualizer_svgs`（`SvgUpdates`）として送信。`activate(node)` を一度呼ぶだけで利用可能です。
- **crane_visualization_aggregator**  
  `/visualizer_svgs` を集約し、5秒ごとに `/aggregated_svgs`（`SvgSnapshot`）を配信。Foxglove・Webビューア・録画再生はこのスナップショット＋差分更新を組み合わせて描画します。

```text
VisualizerMessageBuilder ─┐
                          ├─ CraneVisualizerBuffer → /visualizer_svgs (SvgUpdates)
VisualizerMessageBuilder ─┘             └─ crane_visualization_aggregator → /aggregated_svgs (SvgSnapshot, 5s周期)
```

## 基本セットアップ

```cpp
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("demo_node") {
    crane::CraneVisualizerBuffer::activate(*this);                     // 1回だけ呼べばOK
    visualizer_ = std::make_shared<crane::VisualizerMessageBuilder>("demo_layer");
  }

  void tick(const Point & ball_pos) {
    visualizer_->asReplace();                                          // レイヤーを丸ごと描き直す
    visualizer_->drawFilledCircle(ball_pos, 0.1, "cyan", 0.4);         // 高レベルAPI
    visualizer_->flush();                                              // バッファへ移動
    crane::CraneVisualizerBuffer::publish();                           // /visualizer_svgs へ送信
  }

private:
  crane::VisualizerMessageBuilder::SharedPtr visualizer_;
};
```

- レイヤー名（ここでは `demo_layer`）ごとに描画が管理されます。同じレイヤー名を使うと上書きされます。
- `activate(node, topic)` でカスタムトピックに変更できますが、通常はデフォルトの `/visualizer_svgs` を使います。

## レイヤー操作と送信フロー

1. プリミティブを組み立てる（`circle()`, `line()` など）
2. `flush()` を呼んでメッセージバッファへ移動
3. `CraneVisualizerBuffer::publish()` で更新を送信

レイヤーの操作種別は `operation` で制御します。

```cpp
visualizer->asReplace();  // レイヤー全体を置換（デフォルト）
visualizer->asAppend();   // 既存描画に追記（軌跡など）
visualizer->asClear();    // レイヤーを空にする命令を送信
```

`append` だけを送るとベースが無い場合に無視されるため、初回や定期的に `replace` を挟むと安全です。

## プリミティブビルダー

| メソッド | 主な設定メンバー | 備考 |
|----------|------------------|------|
| `circle()` | `center(x,y)` / `radius(r)` | `fill()` / `stroke()` / `strokeWidth()` など |
| `line()` | `start(p)` / `end(p)` / `fromSegment(segment)` | |
| `polyline()` | `addPoint(p)` / `setPoints(vector)` | |
| `polygon()` | `addPoint(p)` / `setPoints(vector)` | |
| `rect()` | `top_left(p)` / `size(p)` / `box(box)` | |
| `text()` | `position(p)` / `viewBoxPosition(x%,y%)` / `text("label")` / `fontSize()` / `textAnchor()` | |
| `path()` | `definition().moveTo().lineTo().closePath()` など | 自由曲線を構築 |

スタイル指定は `fill(color, alpha)`, `stroke(color, alpha)`, `strokeWidth(width)` で共通に行えます。すべての座標・長さはフィールド座標（m）で指定します。

### RAII（`.raii()`）による自動 `build()`

```cpp
visualizer->circle().raii()
  .center(ball.pos)
  .radius(0.05)
  .fill("red", 0.4)
  .stroke("white")
  .strokeWidth(8.0);
```

`.raii()` を呼ぶとスコープ終了時に自動で `build()` が呼ばれ、書き忘れを防げます。

## 高レベルヘルパー（2025年追加機能）

頻出パターンを1行で描画できます。

- `drawLine(start, end, color = "white", stroke_width = 10.0, opacity = 1.0)`
- `drawCircle(center, radius, color, stroke_width, opacity)`
- `drawFilledCircle(center, radius, fill_color = "white", opacity = 0.5)`
- `drawText(position, "label", color = "white", font_size = 100.0, anchor = "start")`
- `arrow(start, direction, length, color = "white", stroke_width = 10.0, arrowhead_length = 0.35, arrowhead_width = 0.20)`
- `velocityArrow(position, velocity_vector, color = "lime", scale = 1.0, stroke_width = 20.0)`
- `labeledCircle(center, radius, "id", circle_color, text_color, ...)`
- `doubleCircle(center, inner_radius, outer_radius, ...)`
- `rectangle(top_left, bottom_right, color = "white", stroke_width = 10.0)`
- `arc(center, radius, start_angle, end_angle, color = "white", stroke_width = 10.0, steps = 16)`

これらは内部で `add()` 済みなので `build()` は不要です。素早いデバッグに最適です。

## バッファの管理とスナップショット

- `visualizer->clear()` / `clearBuffer()` を使うと未送信の描画を破棄できます。
- `CraneVisualizerBuffer::clear(layer)` / `clear()` は未送信の `SvgUpdates` を削除します。
- `crane_visualization_aggregator` は 5000ms ごとに `/aggregated_svgs` を送信します。巻き戻し再生時に描画が欠ける場合は、この周期を短くするか `replace` を定期的に送ると安定します。

## 実践的なコード例

### ロボット状態の描画

```cpp
void visualizeRobot(const RobotInfo & robot) {
  visualizer->asReplace();
  visualizer->labeledCircle(robot.pose.pos, 0.09, std::to_string(robot.id),
                            crane::SvgColors::Yellow, crane::SvgColors::Black);
  visualizer->arrow(robot.pose.pos, crane::Vector2::fromAngle(robot.pose.theta),
                    0.20, crane::SvgColors::Blue, 12.0);
  visualizer->flush();
  crane::CraneVisualizerBuffer::publish();
}
```

### 軌跡＋最新位置

```cpp
void visualizeTrajectory(const std::vector<Point> & samples) {
  auto & poly = visualizer->polyline();
  poly.stroke("green").strokeWidth(6.0);
  for (const auto & p : samples) {
    poly.addPoint(p);
  }
  poly.build();

  if (!samples.empty()) {
    visualizer->drawFilledCircle(samples.back(), 0.04, "red", 0.7);
  }

  visualizer->flush();
  crane::CraneVisualizerBuffer::publish();
}
```

最新の点だけを追加したい場合は `visualizer->asAppend()` に切り替えて1点だけ送信し、一定周期で `asReplace()` へ戻して基礎データを更新します。

### ヒートマップ表示

```cpp
void visualizeHeatmap(const std::vector<std::vector<double>> & grid,
                      double origin_x, double origin_y, double cell) {
  visualizer->asReplace();
  for (size_t y = 0; y < grid.size(); ++y) {
    for (size_t x = 0; x < grid[y].size(); ++x) {
      double v = std::clamp(grid[y][x], 0.0, 1.0);
      int r = static_cast<int>(255 * v);
      int b = static_cast<int>(255 * (1.0 - v));
      visualizer->rect()
        .top_left(Point(origin_x + x * cell, origin_y + y * cell))
        .size(Point(cell, cell))
        .fill(std::format("rgb({},{},{})", r, 0, b), 0.7)
        .stroke(crane::SvgColors::None)
        .build();
    }
  }
  visualizer->flush();
  crane::CraneVisualizerBuffer::publish();
}
```

## トラブルシューティング

- **描画が出ない**: `flush()` と `publish()` を呼んでいるか確認。`append` だけで更新していないか。レイヤー名の重複にも注意。
- **座標がずれる**: すべてフィールド座標（m）で渡す。内部で1000倍＆Y軸反転してSVG座標に変換されています。
- **時間を巻き戻すと描画が欠ける**: `/aggregated_svgs` のスナップショット間隔が長い可能性。aggregator側のタイマー（5000ms）を短縮する、または `replace` をこまめに送信する。
- **旧フォーマットを送っている**: 旧`svg_primitive_array`系APIを使っていないか確認し、`VisualizerMessageBuilder` に一本化する。

---

`VisualizerMessageBuilder` と `CraneVisualizerBuffer` を活用すれば、ロボット・ボール・戦術の状態を高速に可視化できます。  
レイヤー管理とヒステリシス付き描画を意識しつつ、高レベルヘルパーを使って開発効率を高めてください。
