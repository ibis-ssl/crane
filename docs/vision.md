# SSL-Vision 設定手順

実機環境および大会運用における公式画像処理システム [SSL-Vision](https://github.com/RoboCup-SSL/ssl-vision) の設定と運用のガイドです。

## 構成と役割

- **実機環境**: 外部カメラからの映像を SSL-Vision が処理し、フィールド幾何情報（Geometry）およびボール・ロボット検出情報（Detection）をマルチキャスト配信します。
- **Crane 側の受信**: ポート `10020`（既定値）で受信し、`crane_world_model_publisher` が世界モデルを構築します。
- **Docker 開発環境**: 通常は `ssl-vision-client` コンテナ（ポート `8082`）やシミュレータが Vision データを模擬・中継します。

ビルドとソースは [RoboCup-SSL/ssl-vision](https://github.com/RoboCup-SSL/ssl-vision) を参照してください。

## 設定の流れ

1. 起動とカメラ認識
2. カメラ画像取得
3. フィールド幾何設定
4. カラーキャリブレーション（LUT作成）
5. カメラ幾何キャリブレーション
6. ブロブ（Blob）検出・マーカー認識調整
7. ネットワーク配信確認

## 1. カメラ設定

1. **起動**: リポジトリルートで `./bin/vision` を実行します。
2. **キャプチャ設定**: `Thread0/ImageCapture/Video 4 Linux/CaptureSettings` で以下を設定します。
   - `cam_idx`: カメラデバイス番号（映らない場合は変更して試行）
   - `width` / `height`: 解像度
3. **取り込み開始**: `Thread0/ImageCapture/CaptureControl` で `start capture` を選択します。

## 2. フィールド幾何設定

`Global/FieldConfiguration` で大会規定のフィールド寸法を設定します。

- `Field Length`（長辺） / `Field Width`（短辺）
- `Total Number of Cameras` / `Local Number of Cameras`
- `Number of Line Segments` / `Number of Arcs`

> [!TIP]
> ロボットの検出位置にオフセット（ずれ）が生じる場合は、`Global/Robot Detection/Teams` でロボット高さを調整します（高さを 0 に設定すると改善する場合があります）。

## 3. カラーキャリブレーション

右側の `Auto Color Calibration` タブを使用します。

1. 抽出対象の色を選択します。
2. カメラ画像上で該当色の領域をクリックし、サンプルピクセルを収集します。
3. `Update LUT` を押してルックアップテーブルを更新します。
4. `Thread0/Visualization/threshold` を有効化し、二値化結果を確認します。

## 4. カメラキャリブレーション

1. `Thread0/Visualization/camera calibration` を有効化します。
2. `Camera Calibrator/Calibration Parameters` でコントロールポイントを設定します。フィールド実座標（mm）を入力し、画像上の対応点と関連付けます（右側タブでドラッグ移動も可能）。
3. `Camera Height(in mm)` にカメラの高さを入力します。
4. `Do initial calibration`、続いて `Do full calibration` を実行します。

## 5. パターン認識とマーカー検出

カラー抽出で得られたブロブ（領域の塊）からロボットおよびボールを同定します。

1. **ブロブフィルタ**: `Thread0/Blob Finding` で `min_blob_area`（最小面積）を設定し、ノイズを除去しつつ最小のボールが消えない閾値に調整します。
2. **マーカー認識**: `Global/Robot Detection/Pattern` で中心マーカーおよび個別マーカーの寸法・面積フィルターを設定します。
3. **パターンフィッティング**: `Global/Robot Detection/Pattern/Pattern Fitting` でマッチングスコアの重みを調整します。ブロブが検出されているのにロボットが認識されない場合は `Max Error` を適宜緩和します。

## 実装・運用リファレンス

- 受信側実装: [world_model_publisher.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_world_model_publisher/src/world_model_publisher.cpp)
- 起動引数とポート: [crane.launch.xml](https://github.com/ibis-ssl/crane/blob/develop/crane_bringup/launch/crane.launch.xml)
- [試合チェックリスト](match.md) / [ボールトラッキングシステム](ball_tracking_system.md) / [ネットワーク設定](network.md)
