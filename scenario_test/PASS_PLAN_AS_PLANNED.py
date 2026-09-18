"""PassPlan どおりに蹴り、予定した受け手が予定した受領点で受け取れるかを検証する。

既存の PASS_BUILDUP_STATIC との違い
-----------------------------------
PASS_BUILDUP_STATIC は vision だけを見るので「キック後に最初にボールへ触れたのが
別の yellow なら成功」という機能的成功しか判定できない（pass_helpers.py の docstring）。
誰に渡すつもりだったのかは観測できないため、たまたま別の味方に当たっても SUCCESS になる。

このテストは sidecar (`pass_plan_recorder.py`) 経由で PassPlan を観測し、
**キック検出時点の計画**をラッチして実測と突き合わせる:

1. 計画した `receiver_id` が実際に最初に触れたか（意図した相手に渡ったか）
2. 実接触点が計画 `receive_point` から `RECEIVE_POINT_TOLERANCE` 以内か（予定地点か）

配置は setup_pass_plan_pair を使う。PASS_BUILDUP_STATIC の5機配置では
pass_receive セッションにロボットが回らず（defender が使い切る）、受け手役が
自陣へ戻って受領点が攻撃ハーフから消えるため、このテストには使えない。
理由と実測値は setup_pass_plan_pair の docstring に書いた。

フレーキー対策として 3 試行し 2 回以上で pass とする（既存パステストと同じ方針）。
"""

import math

from field_helpers import Field
from pass_helpers import run_pass_trial, setup_pass_plan_pair

# 実接触点と計画受領点の許容差 [m]。
#
# 現行設計と整合させた値。PassReceiverSession はボールが動き出すと
# 受領点を捨てて実測軌道への最近接点へ切り替える
# （pass_receiver_session.hpp の分岐順。docs/pass.md にも設計意図として明記）。
# そのため実接触点は計画受領点からずれるのが正常で、実測では
# 「計画どおりに実行されたパス」でも 0.27〜1.59 m（中央値およそ0.5m）ばらついた。
# 1.0 m は「計画した地点の近傍で受け取った」と言える上限として置いている。
# 誤差は合否に関わらず毎回出力するので、悪化は数値で追える。
RECEIVE_POINT_TOLERANCE = 1.0

# キック検出時刻から何秒さかのぼって「根拠になった計画」を探すか。
# キックでボールが動き出すと計画は即座に解除される一方、pytest 側の
# キック検出はボール速度がしきい値を超えてからなので遅れる。詳細は
# PassPlanLog.plan_acted_on の docstring。
PLAN_LOOKBACK_SEC = 1.0

# このテストでのキック検出しきい値 [m/s]。
# 既定の 1.5 は、出し手がボールへ寄せる際の小突きも拾ってしまい、
# 本来のパスが出る前に試行が SELF_TOUCH で解決してしまう。
# 計画キック初速は実測で 3.7〜5.7 m/s、シュートは 6 m/s 以上なので、
# 2.5 に上げれば「運び」を除いて「蹴った」だけを見られる。
KICK_DETECT_SPEED = 2.5

TRIALS = 3
REQUIRED_SUCCESSES = 2


def _verdict(trial, plan) -> tuple[str, float]:
    """(判定, 受領点誤差[m]) を返す。誤差が定義できない場合は NaN。

    判定:
      AS_PLANNED        計画どおりの出し手が蹴り、計画どおりの受け手が計画受領点付近で受領
      POINT_OFF         出し手・受け手は計画どおりだが受領点が離れている
      WRONG_KICKER      計画と別の味方が蹴った
      WRONG_RECEIVER    計画と別の味方が受領（機能的成功・意図不一致）
      NO_PLAN           キック時点で有効な計画が観測されなかった
      <trial.outcome>   そもそもパスが成立していない（INTERCEPTED 等）
    """
    if trial.outcome != "SUCCESS":
        return trial.outcome, float("nan")
    # 出し手・受け手が実際に採用するのは PLANNING / BALL_IN_FLIGHT だけ
    # （isUsablePassPlan）。INACTIVE や ABORTED を「計画あり」と数えると、
    # 計画が無いまま蹴った試行を WRONG_RECEIVER と誤分類する。
    if plan is None:
        return "NO_PLAN", float("nan")
    error = math.hypot(
        trial.end_pos[0] - plan["receive_point"]["x"],
        trial.end_pos[1] - plan["receive_point"]["y"],
    )
    if plan["kicker_id"] != trial.kicker_id:
        # 計画の出し手とは別のロボットが蹴った。受け手がたまたま一致していても
        # 「計画どおりに蹴った」とは言えない。
        return "WRONG_KICKER", error
    if plan["receiver_id"] != trial.receiver_id:
        return "WRONG_RECEIVER", error
    if error > RECEIVE_POINT_TOLERANCE:
        return "POINT_OFF", error
    return "AS_PLANNED", error


def test_pass_plan_as_planned(field: Field, pass_plan_log):
    verdicts = []
    for i in range(1, TRIALS + 1):
        trial = run_pass_trial(
            field, setup_pass_plan_pair, kick_detect_speed=KICK_DETECT_SPEED
        )
        plan = (
            pass_plan_log.plan_acted_on(trial.kick_wall_time, PLAN_LOOKBACK_SEC)
            if trial.kick_wall_time
            else None
        )
        verdict, error = _verdict(trial, plan)
        verdicts.append(verdict)

        print(f"試行{i}: {verdict}")
        print(
            f"  観測: outcome={trial.outcome} kicker={trial.kicker_id} "
            f"receiver={trial.receiver_id} 接触点=({trial.end_pos[0]:.2f},{trial.end_pos[1]:.2f}) "
            f"初速={trial.kick_speed:.2f} 距離={trial.pass_distance:.2f}"
        )
        if plan is None:
            # 計画が観測できないのが最も多い失敗。どの段で落ちたかを出す。
            print(
                f"  計画: (キック時点で未観測) 直近={pass_plan_log.describe(float('inf'))}"
            )
        else:
            point = plan["receive_point"]
            print(
                f"  計画: {plan['state_name']} id={plan['plan_id']} "
                f"kicker={plan['kicker_id']} receiver={plan['receiver_id']} "
                f"受領点=({point['x']:.2f},{point['y']:.2f}) 初速={plan['kick_speed']:.2f} "
                f"score={plan['score']:.2f} 到達時間={plan['ball_travel_time']:.2f}s"
            )
            print(f"  受領点誤差: {error:.2f} m (許容 {RECEIVE_POINT_TOLERANCE})")
            ng = pass_plan_log.failing_gates(plan)
            if ng:
                print(f"  NGゲート: {ng}")
            print(f"  可用性: {plan['availability']}")
        assign = (
            pass_plan_log.assign_at(trial.kick_wall_time)
            if trial.kick_wall_time
            else None
        )
        print(f"  役割割当: {assign['watched'] if assign else '(未観測)'}")
        drift = ", ".join(
            f"Y{k}:{v:.2f}" for k, v in sorted(trial.drift_at_kick.items())
        )
        print(f"  キック時の配置ずれ[m]: {drift or '(未計測)'}")

    as_planned = verdicts.count("AS_PLANNED")
    print(f"PASS_PLAN_AS_PLANNED verdicts: {verdicts}")
    assert as_planned >= REQUIRED_SUCCESSES, (
        f"計画どおりの受領 {as_planned}/{TRIALS} (要求: {REQUIRED_SUCCESSES}以上): {verdicts}"
    )
