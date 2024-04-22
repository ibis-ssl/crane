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
より下流で制限するようにしてもいいかも？

### BOT_CRASH_UNIQUE

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_crashing>

#### 概要

#### 対応状況

#### 非対応可能性

### BOT_CRASH_DRAWN

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_crashing>

#### 概要

#### 対応状況

#### 非対応可能性

### ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_too_close_to_opponent_defense_area>

#### 概要

#### 対応状況

#### 非対応可能性

### BOT_TOO_FAST_IN_STOP

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_stop_speed>

#### 概要

#### 対応状況

#### 非対応可能性

### DEFENDER_TOO_CLOSE_TO_KICK_POINT

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_defender_too_close_to_ball>

#### 概要

#### 対応状況

#### 非対応可能性

### BOT_INTERFERED_PLACEMENT

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement_interference>

#### 概要

#### 対応状況

#### 非対応可能性

### BOT_PUSHED_BOT

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_pushing>

#### 概要

#### 対応状況

#### 非対応可能性

### BOT_HELD_BALL_DELIBERATELY

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_holding>

#### 概要

#### 対応状況

#### 非対応可能性

### BOT_TIPPED_OVER

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_tipping_over_or_dropping_parts>

#### 概要

#### 対応状況

#### 非対応可能性
