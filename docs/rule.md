# ルール対応状況

## ファールカウンターを増やすもの

### KEEPER_HELD_BALL

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_keeper_held_ball>

#### 概要

ボールをデフェンスエリアで保持しすぎるとだめ。  
divAは5秒、divBは10まで  
STOPのちフリーキック  

#### 対応状況

ゴールキーパーはボールが止まり次第、ボールを排出するプログラムになっている  

#### 非対応可能性

デフェンスエリアギリギリにボールがあり、近くにロボットが迫っている場合には排出できないかも。  
また、このような状況ではそもそもボールに触れなくなるような挙動もあったかも知れないので要対応。  

### BOUNDARY_CROSSING

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_boundary_crossing>

#### 概要

フィールドの外にボールを蹴っちゃだめ。  
ここで言うフィールドの外とは、ラインの外ではなく木枠の外のこと。  
STOPのちフリーキック  

#### 対応状況

特に対応はしていないが、基本的にチップキックは味方の方向にパスするとき、かつそのパスライン上に敵ロボットが存在する場合に限るので基本的には問題ないと考えている。

#### 非対応可能性

コート端にいる味方にむけてチップキックパスをしたら発生するかも。  
ただ、かなり可能性は低いので対応は現時点で考えていない。  

### BOT_DRIBBLED_BALL_TOO_FAR

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_excessive_dribbling>

#### 概要

いわゆるオーバードリブルと呼ばれるもの。  
1m以上ドリブルすると発生するが、一度でもボールがロボットから離れるとリセットされる。  
インプレイ時のみの違反なので、ボールプレイスメントで行う分には問題ない。  
STOPのちフリーキック

#### 対応状況

JapanOpen2024ではボールプレイスメント以外でドリブルを使う予定はないため問題ないはず。  
もしかすると、ペナルティキックのときに相手ゴール近くまでボールを運ぶときに使うかも知れない。  
参考：<https://github.com/ibis-ssl/crane/issues/246>  

#### 非対応可能性

そもそもドリブルしなければ問題なし。

### ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_attacker_touched_ball_in_opponent_defense_area>

#### 概要

相手ディフェンスエリアでボールに触れると発生する。  
（逆に言えば、侵入だけなら大きくは咎められない？）  
ロボットの一部でもエリアに侵入していれば適用されるため、注意が必要。  

#### 対応状況

コストマップ上で立入禁止エリアに設定してあるので、基本的には発生しないはずである。  

#### 非対応可能性

高速でデフェンスエリア付近のボールに接近した場合、勢い余ってディフェンスエリアに侵入してしまうことが考えられる。  
実際に発生した場合、速度上限を下げたり立入禁止エリアにマージンを設けることで対処予定。  
デフェンスエリアの立ち入り禁止は敵味方を区別していないため、デフェンスエリアに立ち入り許可のあるゴールキーパーが相手ゴールまでキーパーダッシュすれば立ち入る可能性があるが、今の所は味方ディフェンスエリアから出るようなプログラムはない。  

### BOT_KICKED_BALL_TOO_FAST

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_speed>

#### 概要

ボールが速すぎると発生する。  
具体的には6.5m/sになると発生する。  

ただし、そもそも6m/s程度になるとVisionロストするとの情報もある。  

#### 対応状況

ストレートキックは50%程度、チップキックは80%程度のパワーを上限に蹴っている。  
これで特に問題はないはず。  

#### 非対応可能性

ミスって設定していたら発生する。  
よりローカルプランなやセンダーなどの下流で制限するようにしてもいいかも？  

### BOT_CRASH_UNIQUE/BOT_CRASH_DRAWN

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_crashing>

#### 概要

ロボットの衝突時に相対速度が1.5m/sを超えるとより早いロボット側のチームに発生する。  
相対速度のより詳しい計算はルールブック参照。  
速度差が0.3m/s未満だと両成敗になる。

#### 対応状況

一応、デフォルトでロボットとその周辺のマージンを取って立ち入り禁止エリアを設定しているので基本発生しないはず。
ロボットとの距離と相対速度で速度を制限することで追加対応予定。  
<https://github.com/ibis-ssl/crane/issues/296>

#### 非対応可能性

マージンが薄いので速度がでると容易に衝突してしまうかも。  
また、お互いが移動している場合には立入禁止エリアがあまり役に立たない。  
また、ディフェンダーやアタッカーなど敵ロボットに忖度していては仕事にならないロボットは立ち入り禁止設定を解除しているので、ぶつかりやすい。

### ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_too_close_to_opponent_defense_area>

#### 概要

STOP中やフリーキック中に相手ディフェンスエリア＋マージン0.2mの中に侵入すると発生する。  
脱出時間として、判定には2秒の猶予があるらしい。

#### 対応状況

現状これに特化したプログラムはないが、基本的にはディフェンスエリアに入るようなプログラムはないはず。

#### 非対応可能性

特に対策しているわけではないので、特定の状況で発生するかも。  
例えば、マークしている敵ロボットがディフェンスエリアに逃げ込んだ場合など  

### BOT_TOO_FAST_IN_STOP

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_stop_speed>

#### 概要

STOP中に1.5m/sを超える速度で移動すると発生する。  
STOPになってから2秒間は猶予時間がある。  

#### 対応状況

現状、STOP中には最大速1.0m/sのフォーメーションが割り当てられている。
ただし、今後はSTOP中により意味ある動きをするプログラムに書き換えるため、  
それに伴ってローカルプランナなどで一括速度制限をかける予定である。 ([#297](https://github.com/ibis-ssl/crane/issues/297))

#### 非対応可能性

[#297](https://github.com/ibis-ssl/crane/issues/297)で完全対応予定

### DEFENDER_TOO_CLOSE_TO_KICK_POINT

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_defender_too_close_to_ball>

#### 概要

敵キックオフやフリーキックの間、ボールから0.5m以上離れていないと発生する。

#### 対応状況

特に対応できていない。  
[#283](https://github.com/ibis-ssl/crane/issues/283)で対応予定。  

#### 非対応可能性

[#283](https://github.com/ibis-ssl/crane/issues/283)で完全対応予定。

### BOT_INTERFERED_PLACEMENT

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement_interference>

#### 概要

ボールプレイスメント中にボールとターゲットで構成される線分への距離が0.5m以内にロボットがいると発生する。

#### 対応状況

立入禁止エリアを設定しているので基本的には発生しないはず。  
これに侵入するように設定しているのは、自チームのボールプレイスメント時のボールハンドラのみである。

#### 非対応可能性

ロボットの目標地点が禁止エリア内にある場合、ロボットは多少ゆらゆらすることがある。  
もう少しマージンを取るようにしてもいいかもしれない。  
また、ボールプレイスメントエリアの配置によってはうまく機能しないかもしれない。  
（壁際でエリアに追い詰められた場合など）  

### BOT_PUSHED_BOT

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_pushing>

#### 概要

ロボットが押し合って、相手のロボットを動かすと発生する。  
押し勝っていると思われるチームに発生する。  
STOPのちフリーキック、AutoRefによる判定はない。

#### 対応状況

特に対応できていない。

#### 非対応可能性

ボールの中心に移動し続けるようなプログラムが何箇所かあり、それが原因で発生する可能性がある。

### BOT_HELD_BALL_DELIBERATELY

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_holding>

#### 概要

ホールディングと呼ばれるもの。  
ボールをロボットで囲み、敵のアプローチを防ぐような行為を行うことで発生する。  
STOPのちフリーキック、AutoRefによる判定はない。

#### 対応状況

特に対応はしていない。

#### 非対応可能性

故意に固まるようなプログラムはないが、特にデフェンス時に似た状況に陥りやすいかもしれない。

### BOT_TIPPED_OVER

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_tipping_over_or_dropping_parts>

#### 概要

ロボットが部品を落としたり、倒れたりすると発生する。  
違反ロボットは交代する必要がある。  

#### 対応状況

ネジ締めなどをしっかりする。  
また、かなり低重心なので転倒はするほうが難しい。

#### 非対応可能性

ネジの緩みなどで発生する可能性がある。
