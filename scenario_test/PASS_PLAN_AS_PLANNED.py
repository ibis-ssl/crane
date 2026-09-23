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

import dataclasses
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
#
# 計画キック初速の実測下限（2.08 m/s。受領点が近く desired_arrival_speed が
# 1.5 のため）より下に置くこと。上回る値（例: 2.5）にすると計画どおりのパスを
# 一度も検出できず、代わりに運び中の小突きを拾って WRONG_RECEIVER と誤判定する。
# 運びの誤検出は分離速度（pass_helpers.KICK_SEPARATION_SPEED）で弾く。
KICK_DETECT_SPEED = 1.6

TRIALS = 3
REQUIRED_SUCCESSES = 2


# robocup_ssl_msgs/Team: UNKNOWN=0, YELLOW=1, BLUE=2。crane は Yellow で起動する。
TRACKER_TEAM_YELLOW = 1


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
        # Tracker がキックを出していれば、その時刻を基準に計画をラッチする。
        # 自前検出はボール速度がしきい値を越えるまで待つぶん必ず遅れるので、
        # 遅れた時刻でさかのぼると、実際に使われた計画ではなく、その後に
        # 差し替わった計画を拾うことがある。実測でそれが起き、受け手が
        # 食い違ったように見えた（計画は受け手10、ボールは受け手2へ）。
        # Tracker の kicked_ball は optional で来ないことがあるため、
        # 来ないときは従来どおり自前検出の時刻を使う。
        tracker_kick = (
            pass_plan_log.tracker_kick_near(trial.kick_wall_time)
            if trial.kick_wall_time
            else None
        )
        kick_time = (
            pass_plan_log.kick_time(tracker_kick)
            if tracker_kick
            else trial.kick_wall_time
        )
        plan = (
            pass_plan_log.plan_acted_on(kick_time, PLAN_LOOKBACK_SEC)
            if kick_time
            else None
        )
        # キックの分類は EKF 由来の速度で行う。vision の位置差分は外れ値が出る。
        ekf_peak = (
            pass_plan_log.ball_peak_speed(kick_time - 0.3, kick_time + 0.8)
            if kick_time
            else float("nan")
        )
        # 出し手も Tracker が答えを持っている。自前検出は「ボールが速くなった
        # 直前に最も近かった味方」なので、受け手が触った瞬間を拾うと出し手を
        # 取り違える。実測でそれが起き、計画どおりの試行が WRONG_KICKER に
        # 化けた。team=1 は yellow（crane 側）。
        judged = trial
        if tracker_kick and tracker_kick["team"] == TRACKER_TEAM_YELLOW:
            robot_id = tracker_kick["robot_id"]
            if robot_id is not None:
                judged = dataclasses.replace(trial, kicker_id=robot_id)
        verdict, error = _verdict(judged, plan)
        verdicts.append(verdict)

        print(f"試行{i}: {verdict}")
        print(
            f"  観測: outcome={trial.outcome} kicker={trial.kicker_id} "
            f"receiver={trial.receiver_id} 接触点=({trial.end_pos[0]:.2f},{trial.end_pos[1]:.2f}) "
            f"初速={trial.kick_speed:.2f}(回帰{trial.kick_speed_fit:.2f} "
            f"EKF{ekf_peak:.2f}) 距離={trial.pass_distance:.2f} "
            # 計画側は yaml の減速度を前提に初速を決める。ここに出るのは
            # シミュレータの実測値なので、両者がずれていれば設定が効いて
            # いないか較正が古い。計画値との突き合わせに必須。
            f"実測減速度={trial.ball_decel_fit:.2f}"
        )
        if tracker_kick is None:
            print("  Tracker: キック検出なし（生成側が kicked_ball を出していない）")
        else:
            print(
                f"  Tracker: kicker={tracker_kick['robot_id']}(team={tracker_kick['team']}) "
                f"初速={tracker_kick['speed']:.2f} vz={tracker_kick['vz']:.2f} "
                f"開始点=({tracker_kick['x']:.2f},{tracker_kick['y']:.2f})"
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
