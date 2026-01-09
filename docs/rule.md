# ルール違反対応状況

> **最終更新**: 2025年6月22日（JapanOpen2025後）  
> **対応ROS**: ROS 2 Jazzy  
> **関連パッケージ**: crane_local_planner, crane_physics, crane_robot_skills

このドキュメントでは、RoboCup SSL公式ルールにおける各種ルール違反への対応状況をまとめています。特にファールカウンターを増やす違反について、現在の実装状況と今後の対応方針を記載します。

## KEEPER_HELD_BALL

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_keeper_held_ball>

### 概要

ボールをディフェンスエリアで保持しすぎるとだめ。  
divAは5秒、divBは10まで  
STOPのちフリーキック  

### 対応状況

ゴールキーパーはボールが止まり次第、ボールを排出するプログラムになっている  

### 非対応可能性

ディフェンスエリアギリギリにボールがあり、近くにロボットが迫っている場合には排出できないかも。  
また、このような状況ではそもそもボールに触れなくなるような挙動もあったかも知れないので要対応。  

## BOUNDARY_CROSSING

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_boundary_crossing>

### 概要

フィールドの外にボールを蹴っちゃだめ。  
ここで言うフィールドの外とは、ラインの外ではなく木枠の外のこと。  
STOPのちフリーキック  

### 対応状況

特に対応はしていないが、基本的にチップキックは味方の方向にパスするとき、かつそのパスライン上に敵ロボットが存在する場合に限るので基本的には問題ないと考えている。

### 非対応可能性

コート端にいる味方にむけてチップキックパスをしたら発生するかも。  
ただ、かなり可能性は低いので対応は現時点で考えていない。  

## BOT_DRIBBLED_BALL_TOO_FAR

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_excessive_dribbling>

### 概要

いわゆるオーバードリブルと呼ばれるもの。  
1m以上ドリブルすると発生するが、一度でもボールがロボットから離れるとリセットされる。  
インプレイ時のみの違反なので、ボールプレイスメントで行う分には問題ない。  
STOPのちフリーキック

### 対応状況

現在の実装では、`crane_robot_skills`の`Dribble`スキルにおいて、ドリブル距離のトラッキング機能を実装済み。  
ボールプレイスメント時以外でのドリブル使用は自動的に距離制限（0.8m以内）がかかるようになっている。  
ペナルティキック時の特別処理も`PenaltyKick`スキルで対応済み。  

### 非対応可能性

そもそもドリブルしなければ問題なし。

## ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_attacker_touched_ball_in_opponent_defense_area>

### 概要

相手ディフェンスエリアでボールに触れると発生する。  
（逆に言えば、侵入だけなら大きくは咎められない？）  
ロボットの一部でもエリアに侵入していれば適用されるため、注意が必要。  

### 対応状況

`crane_local_planner`のRVO2ベース衝突回避システムにおいて、ディフェンスエリアは立入禁止エリアとして設定済み。  
さらに、`AvoidDefenseArea`スキルが全ロボットに適用され、エリアからの自動退避機能も実装されている。  

### 非対応可能性

高速でディフェンスエリア付近のボールに接近した場合、勢い余ってディフェンスエリアに侵入してしまうことが考えられる。  
実際に発生した場合、速度上限を下げたり立入禁止エリアにマージンを設けることで対処予定。  
ディフェンスエリアの立ち入り禁止は敵味方を区別していないため、ディフェンスエリアに立ち入り許可のあるゴールキーパーが相手ゴールまでキーパーダッシュすれば立ち入る可能性があるが、今の所は味方ディフェンスエリアから出るようなプログラムはない。  

## BOT_KICKED_BALL_TOO_FAST

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_speed>

### 概要

ボールが速すぎると発生する。  
具体的には6.5m/sになると発生する。  

ただし、そもそも6m/s程度になるとVisionロストするとの情報もある。  

### 対応状況

ストレートキックは50%程度、チップキックは80%程度のパワーを上限に蹴っている。  
これで特に問題はないはず。  

### 非対応可能性

ミスって設定していたら発生する。  
よりローカルプランなやセンダーなどの下流で制限するようにしてもいいかも？  

## BOT_CRASH_UNIQUE/BOT_CRASH_DRAWN

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_crashing>

### 概要

ロボットの衝突時に相対速度が1.5m/sを超えるとより早いロボット側のチームに発生する。  
相対速度のより詳しい計算はルールブック参照。  
速度差が0.3m/s未満だと両成敗になる。

### 対応状況

`crane_local_planner`のRVO2アルゴリズムにより、マルチロボット環境での分散的衝突回避を実現。  
各ロボットは半径0.06m + 安全マージン0.1mの回避領域を持ち、相対速度に基づいた動的速度制限も実装済み。  
`crane_physics`の移動時間計算により、安全な加速度プロファイルで制御されている。  

### 非対応可能性

マージンが薄いので速度がでると容易に衝突してしまうかも。  
また、お互いが移動している場合には立入禁止エリアがあまり役に立たない。  
また、ディフェンダーやアタッカーなど敵ロボットに忖度していては仕事にならないロボットは立ち入り禁止設定を解除しているので、ぶつかりやすい。

## ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_too_close_to_opponent_defense_area>

### 概要

STOP中やフリーキック中に相手ディフェンスエリア＋マージン0.2mの中に侵入すると発生する。  
脱出時間として、判定には2秒の猶予があるらしい。

### 対応状況

現状これに特化したプログラムはないが、基本的にはディフェンスエリアに入るようなプログラムはないはず。

### 非対応可能性

特に対策しているわけではないので、特定の状況で発生するかも。  
例えば、マークしている敵ロボットがディフェンスエリアに逃げ込んだ場合など  

## BOT_TOO_FAST_IN_STOP

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_stop_speed>

### 概要

STOP中に1.5m/sを超える速度で移動すると発生する。  
STOPになってから2秒間は猶予時間がある。  

### 対応状況

`crane_tactic_coordinator`のプレイ状況管理により、STOP中は全ロボットに速度制限（1.0m/s）が適用される。  
`crane_local_planner`レベルでも追加の速度制限が実装されており、STOP検出から2秒間の猶予時間中に  
段階的に減速する仕組みが導入済み。

### 非対応可能性

現在の実装で十分対応可能。JapanOpen2025でも違反は確認されていない。

## DEFENDER_TOO_CLOSE_TO_KICK_POINT

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_defender_too_close_to_ball>

### 概要

敵キックオフやフリーキックの間、ボールから0.5m以上離れていないと発生する。

### 対応状況

`crane_tactics`の`DefenseFormation`において、敵フリーキック時の防御位置計算を実装済み。  
ボールから0.5m + 安全マージン0.1mの距離を保つように、全ディフェンダーの配置が自動計算される。  
`KeepDistance`スキルにより、指定距離の維持も可能。

### 非対応可能性

極めて低い。実装済みのアルゴリズムで十分対応可能。

## BOT_INTERFERED_PLACEMENT

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement_interference>

### 概要

ボールプレイスメント中にボールとターゲットで構成される線分への距離が0.5m以内にロボットがいると発生する。

### 対応状況

`BallPlacement`スキルにおいて、ボール-ターゲット間の線分から0.5m + マージン0.1mの回避エリアを動的に設定。  
`crane_local_planner`が全ロボットに対してこのエリアを回避するように経路計算する。  
自チームのボールプレイスメント実行ロボットのみ、専用の接近経路を使用。

### 非対応可能性

ロボットの目標地点が禁止エリア内にある場合、ロボットは多少ゆらゆらすることがある。  
もう少しマージンを取るようにしてもいいかもしれない。  
また、ボールプレイスメントエリアの配置によってはうまく機能しないかもしれない。  
（壁際でエリアに追い詰められた場合など）  

## BOT_PUSHED_BOT

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_pushing>

### 概要

ロボットが押し合って、相手のロボットを動かすと発生する。  
押し勝っていると思われるチームに発生する。  
STOPのちフリーキック、AutoRefによる判定はない。

### 対応状況

特に対応できていない。

### 非対応可能性

ボールの中心に移動し続けるようなプログラムが何箇所かあり、それが原因で発生する可能性がある。

## BOT_HELD_BALL_DELIBERATELY

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_holding>

### 概要

ホールディングと呼ばれるもの。  
ボールをロボットで囲み、敵のアプローチを防ぐような行為を行うことで発生する。  
STOPのちフリーキック、AutoRefによる判定はない。

### 対応状況

`crane_tactics`の戦術プランナーにおいて、ロボット間の最小距離（0.3m）を保つ制約を実装。  
ボール周辺への過度な集中を避けるため、各ロボットに明確な役割分担と位置制約を設定している。

### 非対応可能性

ディフェンス時の密集陣形では、意図しないホールディング状況が発生する可能性が低確率で存在する。

## BOT_TIPPED_OVER

<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_tipping_over_or_dropping_parts>

### 概要

ロボットが部品を落としたり、倒れたりすると発生する。  
違反ロボットは交代する必要がある。  

### 対応状況

ネジ締めなどをしっかりする。  
また、かなり低重心なので転倒はするほうが難しい。

### 非対応可能性

ネジの緩みなどで発生する可能性がある。

---

## JapanOpen2025での実績

### 違反発生状況

- **発生した違反**: BOT_KICKED_BALL_TOO_FAST（1回）、BOT_CRASH_UNIQUE（2回）
- **未発生**: その他の主要な違反は発生せず、対応策が有効に機能

### 効果的だった対応

1. **RVO2衝突回避**: マルチロボット環境での安全性が大幅に向上
2. **動的速度制限**: STOP時やディフェンスエリア付近での制御が安定
3. **自動距離管理**: フリーキック時の自動退避が効果的

### 今後の改善点

- キック力制限の更なる精密化（6.5m/s制限に対してより安全なマージン設定）
- 高速移動時の衝突予測アルゴリズムの改良
- より保守的な安全マージンの設定検討

### 関連パッケージ

詳細な実装については以下のパッケージドキュメントを参照：

- [crane_local_planner](./packages/crane_local_planner.md) - RVO2衝突回避システム
- [crane_robot_skills](./packages/crane_robot_skills.md) - 各種ルール対応スキル
- [crane_physics](./packages/crane_physics.md) - 物理制約と動力学計算
- [crane_tactic_coordinator](./packages/crane_tactic_coordinator.md) - ゲーム状況管理
