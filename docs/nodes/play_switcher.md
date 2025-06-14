# Play Switcher

`PlaySwitcher`は、ROS 2ノードとして動作し、主にSSL Refereeからの信号 (`/referee`トピック) を受け取ります。これらの信号を解析し、現在のプレイ状況 (`/play_situation`トピック) を他のノードに配信します。Refereeからは毎フレーム情報が流れてくる可能性がありますが、PlaySwitcherはプレイ状況に実質的な変更があった場合にのみ新しい情報をパブリッシュするように設計されています。

このノードは、`/world_model`トピックからボールの位置情報などを参照し、INPLAY状態の判定など、Referee情報だけでは直接得られない高度なゲーム状態の解釈も行います。

## 主な機能

### 敵味方イベントの解釈

RefereeからのイベントはBlueチームまたはYellowチームに対して発行されます。PlaySwitcherは、設定された`team_name`パラメータ（詳細は後述）に基づいて、これらのイベントを自チーム（Our）と相手チーム（Their）の観点からのイベントに変換します。

## NORMAL_STARTの解釈

前のイベントに応じてNORMAL_STARTの解釈を行う

参考：[5.3.1 Normal Start | Official Rule](https://robocup-ssl.github.io/ssl-rules/sslrules.html#_normal_start)

### NORMAL_STARTの解釈

| 前のイベント          | `NORMAL_START`の解釈 |
| --------------------- | -------------------- |
| `KICKOFF_PREPARATION` | `KICKOFF_START`      |
| `PENALTY_PREPARATION` | `PENALTY_START`      |

## INPLAYの判定

INPLAYのイベントはRefereeからは送信されてこないのでPlaySwitcherで判定します。この判定は、`/world_model`からのボール情報も利用します。

参考：[5.4. Ball In And Out Of Play | Official Rule](https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_in_and_out_of_play)

### INPLAYの判定条件

- `FORCE_START`コマンド受信時。
- 以下のイベント後，ボールが少なくとも0.05m動いた時（ボールの位置は`/world_model`から取得）：
  - `KICKOFF_START`
  - `PENALTY_START`
  - `DIRECT_FREE`
  - `INDIRECT_FREE`
- フリーキック（`THEIR_DIRECT_FREE`）後、約12.0秒が経過した時。
- キックオフ（`THEIR_KICKOFF_START`）後、約10.0秒が経過した時。

## STOPコマンドの解釈

STOPコマンドは、次に予定されているコマンド（`next_command`）に応じて、より詳細な状態（例：`STOP_PRE_OUR_PENALTY_PREPARATION`）に解釈されます。`next_command`が指定されていない場合は、汎用的な`STOP`状態となります。この機能は実装済みです。

## ROSインターフェース

### サブスクライブするトピック

- `/referee` (`robocup_ssl_msgs::msg::Referee`)
  - SSL Refereeからの公式な試合状態コマンドを受信します。
- `/world_model` (`crane_msgs::msg::WorldModel`)
  - `WorldModelWrapper`を通じて購読。主にボールの位置や速度情報をINPLAY判定に使用します。
- `/session_injection` (`std_msgs::msg::String`)
  - デバッグやテスト目的で、特定のプレイ状況を一時的に注入するために使用されます。

### パブリッシュするトピック

- `/play_situation` (`crane_msgs::msg::PlaySituation`)
  - 解析・解釈された現在のプレイ状況を配信します。状態に変化があった場合のみパブリッシュされます。
- `~/process_time` (`std_msgs::msg::Float32`)
  - Refereeメッセージ処理にかかった時間をパブリッシュします（デバッグ用）。

### ROSパラメータ

- `team_name` (string, default: "ibis")
  - 自チームの名称。Refereeメッセージ内のBlue/Yellowチーム情報を、自チーム/相手チーム情報に変換する際に使用されます。
