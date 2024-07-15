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

※ロボットがオフセットしているように感じたら...

- 「Global/Robot Detection/BlueTeam」などからチームを確認
- 「Global/Robot Detection/Teams/ER-Force」などからロボットの高さを調整
  ロボットの高さをゼロにするとオフセットがなくなることがある

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

各スレッドの「Camera Calibrator/Calibration Parameters(list)」以下でコントロールポイントの設定ができる。  
それぞれのコントロールポイントのフィールドのxyの座標をmm単位で設定した後、カメラ画像上の点と対応付ける。
座標を直に設定してもよいが、右の「Camera Calibration」タブを開いた状態にすると、コントロールポイントをドラッグして移動することができる。

### キャリブレーションの実行

コントロールポイントを設定したら、キャリブレーションを実行する。
まず、右の「Camera Calibration」タブを開いて一番下の「Initial Camera Parameters」にある「Camera Height(in mm)」にカメラの高さを設定する。
次に、「Do initial calibration」「Do full calibration」の順にボタンを押してキャリブレーションを実行する。

## パターン認識の設定

色が設定できると、「blob」と呼ばれる画像上の同色の塊が認識されるようになる。
認識されている様子は各スレッドの「Visualization/blobs」にチェックを入れることで確認できる。
SSL-Visionでは、このblobに対してパターン認識を行い、ロボットやボールを認識する。

### blobのフィルタリング

各スレッドの「Blob Finding」にて認識するblobの最小面積(単位：ピクセル)「min_blob_area」や最大認識数「max regions」などを設定できる。

「min_blob_area」は認識物体の中で一番小さいボールが消えない程度に設定するとよいだろう。

### 各マーカーの認識設定

「Global/Robot Detection/Pattern」の「Center Marker」や「Other Markers」でマーカーの認識調整が行える。
ここでは認識されるマーカーの（画像上の）最小・最大の幅・高さ・面積のフィルターを設定できる。

### パターンマッチングの設定

「Global/Robot Detection/Pattern/Pattern Fitting」でパターンマッチングの設定が行える。
マッチングスコアの重みを調整することができる。
blobが認識できているのに、ロボットが認識されない場合は、「Max Error」を大きくしてみて見るのも良いだろう。

