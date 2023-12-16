# 現状でできること

## CONSAI由来のパッケージ郡

### `robocup_ssl_comm`

- GameControllerの通信内容をROSのメッセージに変換する
  - `HALT`などのRefBoxコマンド
  -
- GrSimの通信内容をROSのメッセージに変換する
  - Visionとは別の以下のようなコマンド
    - ロボット・ボールの移動（神の手）
    - ロボットの移動（AIからの司令送信）
- Visionの通信内容をROSのメッセージに変換する

### `consai_vision_tracker`

Visionの情報からボール・ロボットの位置を推定する

### `consai_visualizer`

特に手を加えていない

## crane |

### `crane_sender`

### `crane_world_model_publisher`

## crane | session

craneでは、セッションという単位でロボットの動作を管理する
セッションでは，複数のロボットがそれぞれの役割・目的が与えられる
このとき，ロボットには役割に応じて用意してある専用のプランナが割り当てられる
※このプランナ郡は `crane_planner_plugins` にある
例えば，ある攻撃状態でパス中のセッションは以下のようになる

- ロボットA：goalie planner
- ロボットB：defender planner
- ロボットC：receiver planner
  - パスを受けるロボット
- ロボットD：waiter planner
  - 次のパスを受ける地点へ移動するロボット
- ロボットE：marker planner
  - パスカットを邪魔するロボット

これらはファールなどでロボットの数が可変になる中でもうまく割り当てを行う必要があるため，割当プランナが存在する

セッションには，優先順位・最大ロボット使用台数と共にプランナが登録される．
`session_controller`は今使えるロボットをこの優先順位に従ってロボットを配置する．
`session_controller`は，優先順位が高いプランナから順に，今空いているロボットの情報を送信し，プランナにとって最も都合の良いロボットを選択してもらう．
これを使えるロボットがなくなるまで繰り返してロボットの割当を行う．

### `crane_planner_base`

### `crane_planner_plugins`

- ball_placement
  - 現状
    - ドリブルによるボールの配置．多分まだ動かない
  - 今後
    - パスによるボールの配置を行う
    - ボールが壁によりすぎてしまった場合に壁の反発を利用して取得する
- defender
  - 現状
    - ゴールエリア沿いに壁を作る
      - ロボット数可変対応済み
  - 今後
    - コーナーキックなど，敵の配置に応じて壁を作る
- goalie
  - 現状
    - シュートのブロッキングポイントに移動する
    - シュートがないときは，ボールとゴールの間に位置する
  - 今後
    - defenderとの連携を行う
    - ゴールエリアにボールが来た場合の処理
- receive
- waiter
  - 現状
    - ただ待つだけ
  - 今後
    - 指定された位置・エリアに移動する
      - game analyzerと連携して目的に応じて有利な位置へのポジショニングをやりたい
- player

### `crane_session_controller`
