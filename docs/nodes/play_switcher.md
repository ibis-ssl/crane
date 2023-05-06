# Play Switcher

refereeからの信号を受け取って，解析した情報を流すノード  
refereeからは毎フレーム情報が流れてくるが，PlaySwitcherからの出力時は情報に更新があった時のみ

## 敵味方イベントの解釈

Blue / Yellowのイベントを解釈して，敵味方のイベントに変換する

## NORMAL_STARTの解釈

前のイベントに応じてNORMAL_STARTの解釈を行う

参考：[5.3.1 Normal Start | Official Rule](https://robocup-ssl.github.io/ssl-rules/sslrules.html#_normal_start)

### NORMAL_STARTの解釈

| 前のイベント                | `NORMAL_START`の解釈 |
|-----------------------|-------------------|
| `KICKOFF_PREPARATION` | `KICKOFF_START`   |
| `PENALTY_PREPARATION` | `PENALTY_START`   |

## INPLAYの判定

INPLAYのイベントはRefereeからは送信されてこないのでPlaySwitcherで判定する

参考：[5.4. Ball In And Out Of Play | Official Rule](https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_in_and_out_of_play)
### INPLAYの判定

- `FORCE_START`発動時
- 以下のイベント後，ボールが少なくとも0.05m動いた時
  - `KICKOFF_START`
  - `PENALTY_START`
  - `DIRECT_FREE`
  - `INDIRECT_FREE`
- フリーキックで以下の秒数が経過したとき
  - 5秒（Division A）
  - 10秒（Division B）
