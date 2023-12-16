# SSL-Vision

## 環境構築

```bash
git clone git@github.com:RoboCup-SSL/ssl-vision.git
cd ssl-vision
mkdir build
cd build
cmake .. -DUSE_V4L=true
make -j
cd ..
./bin/vision
```

## 設定の流れ

1. 起動
2. カメラの設定
3. フィールドの設定
4. 色の設定
5. カメラキャリブレーション
6. マスクの設定
7. Blobの設定
8. ボール・ロボット認識の設定
9. ネットワークの設定

## 起動

リポジトリのルートディレクトリで以下のコマンドを実行する．

```bash
./bin/vision
```

## カメラの設定

### 前設定

Thread0の「ImageCapture/Video 4 Linux/CaptureSettings」で以下を設定

- cam_idx
  - カメラが映らなかったらここのIDを変えてみる
- width
- height

### 映す

Thread0の「ImageCapture/CaptureControl」の「start capture」

## フィールドの設定

「Global/FieldConfiguration」を設定する
特に以下を設定

- Field Length(こっちが長辺)
- Field Width
- Total Number of Cameras
- Local Number of Cameras
- Number of Line Segments
- Number of Arcs

## 色の設定

右側の「Auto Color Calibration」タブを使って設定する

1. 色を選択する（その色でサンプルを取得するモードになる）
2. 画像上で選択した色のピクセルをいくつかクリックする（サンプルされる）
3. 「Update LUT」ボタンを押す

### 調整・確認方法など

- 色の識別結果の可視化
  - 「Thread0/Visualization/threshold」をTrueに設定する
- 取得したサンプルを削除する
  - 1 「Remove all samples」ボタンを押す
    - 少し不安定でこれを押すとVisionが落ちることも
  - 2 「Thread0/Auto Color Calibration/Calibration Points」
    - ここにサンプルが全て列挙されているのでクリックして中にある「remove」ボタンで削除できる

## カメラキャリブレーション

「Thread0/Visualization/camera calibration」をTrueにする

### コントロールポイントの設定
