"""測定器そのものの自己テスト（シミュレータ不要・数秒で終わる）。

PassPlan の検証は sidecar が書く JSONL の読み取りと判定ロジックに全面的に依存する。
ここが間違っていると、シミュレータを何回回しても得られる数字が意味を持たない。
実際に開発中、読み取り側で「書きかけの行を読み飛ばす」バグと、判定側で
「INACTIVE を計画ありと数える」「出し手が一致しているか見ていない」バグを出した。
同じ壊れ方を二度しないよう、合成データで固定する。

Docker もクレーンも要らないので `pytest scenario_test/pass_plan_log_selftest.py` で
単独実行できる（`TEST=all` でも収集される。pytest.ini が python_files = *.py）。
"""

import json
import math
from types import SimpleNamespace

from PASS_PLAN_AS_PLANNED import RECEIVE_POINT_TOLERANCE, _verdict
from pass_plan_log import PassPlanLog


def _plan(t, state_name, *, plan_id=1, kicker=1, receiver=2, x=1.0, y=0.0):
    return {
        "kind": "plan",
        "t": t,
        "plan_id": plan_id,
        "state_name": state_name,
        "kicker_id": kicker,
        "receiver_id": receiver,
        "receive_point": {"x": x, "y": y},
        "kick_speed": 3.0,
        "score": 0.4,
        "ball_travel_time": 1.2,
        "gates": {"inplay": True, "score_ok": True},
        "availability": {},
    }


def _write(path, records, *, trailing_partial=False):
    with open(path, "w", encoding="utf-8") as fp:
        fp.writelines(json.dumps(record) + "\n" for record in records)
        if trailing_partial:
            # 改行の来ていない書きかけの行。recorder は行バッファなので実際に起こる。
            fp.write('{"kind": "plan", "t": 99')


def _trial(outcome="SUCCESS", *, kicker=1, receiver=2, end_pos=(1.0, 0.0)):
    return SimpleNamespace(
        outcome=outcome, kicker_id=kicker, receiver_id=receiver, end_pos=end_pos
    )


# ─── 読み取り ────────────────────────────────────────────────────────────────


def test_poll_holds_back_incomplete_line(tmp_path):
    """書きかけの行は取り込まず、完成してから読む。

    以前は `for line in fp` で読んでおり、先読みバッファのせいで tell() が使えず、
    書きかけの行をそのまま JSON として食って壊れていた。
    """
    path = tmp_path / "log.jsonl"
    _write(path, [_plan(1.0, "PLANNING")], trailing_partial=True)
    log = PassPlanLog(str(path))
    assert len(log.poll()) == 1

    # 書きかけの行が完成したら、次の poll で読める。
    with open(path, "a", encoding="utf-8") as fp:
        fp.write(
            ', "state_name": "PLANNING", "plan_id": 2, "kicker_id": 1, '
            '"receiver_id": 2, "receive_point": {"x": 1.0, "y": 0.0}, '
            '"kick_speed": 3.0, "score": 0.4, "ball_travel_time": 1.2, '
            '"gates": {}, "availability": {}}\n'
        )
    records = log.poll()
    assert len(records) == 2
    assert records[1]["plan_id"] == 2


def test_latest_before_returns_state_in_effect(tmp_path):
    """recorder は変化時だけ書くので、直前の行がその時刻の状態を表す。"""
    path = tmp_path / "log.jsonl"
    _write(path, [_plan(1.0, "PLANNING", plan_id=1), _plan(3.0, "INACTIVE", plan_id=2)])
    log = PassPlanLog(str(path))
    assert log.plan_at(2.0)["plan_id"] == 1
    assert log.plan_at(3.5)["plan_id"] == 2
    assert log.plan_at(0.5) is None


# ─── キック時点の計画のラッチ ────────────────────────────────────────────────


def test_plan_acted_on_ignores_unusable_states(tmp_path):
    """INACTIVE / ABORTED は「計画あり」ではない。

    出し手・受け手が実際に採用するのは PLANNING / BALL_IN_FLIGHT だけ
    （isUsablePassPlan）。ここを緩めると、計画が無いまま蹴った試行が
    WRONG_RECEIVER に化けて「計画はあったが相手が違った」と誤読される。
    """
    path = tmp_path / "log.jsonl"
    _write(path, [_plan(9.0, "INACTIVE"), _plan(9.5, "ABORTED")])
    log = PassPlanLog(str(path))
    assert log.plan_acted_on(10.0, 1.0) is None


def test_plan_acted_on_takes_last_usable_within_lookback(tmp_path):
    """キック検出は遅れるので、さかのぼって最後に有効だった計画を採る。"""
    path = tmp_path / "log.jsonl"
    _write(
        path,
        [
            _plan(8.0, "PLANNING", plan_id=1),  # lookback の外
            _plan(9.6, "PLANNING", plan_id=2),  # これが採られる
            _plan(9.9, "INACTIVE", plan_id=3),  # キックで解除された
        ],
    )
    log = PassPlanLog(str(path))
    assert log.plan_acted_on(10.0, 1.0)["plan_id"] == 2
    # lookback を縮めれば 9.6 も外れる。
    assert log.plan_acted_on(10.0, 0.2) is None


# ─── Tracker のキック検出 ────────────────────────────────────────────────────


def _tracker_kick(t, *, robot=1, team=1, speed=2.0, start=None):
    return {
        "kind": "tracker_kick",
        "t": t,
        "start_timestamp": t if start is None else start,
        "x": 0.0,
        "y": 0.0,
        "vx": speed,
        "vy": 0.0,
        "vz": 0.0,
        "speed": speed,
        "robot_id": robot,
        "team": team,
    }


def test_tracker_kick_is_taken_from_before_the_detection(tmp_path):
    """検出時刻より後のキックは採らない。

    自前のキック検出はボール速度がしきい値を越えるまで待つので必ず遅れる。
    前後で最も近いものを採ると、受け手が触った瞬間のキックを拾って出し手を
    取り違える。実測でそれが起き、計画どおりの試行が WRONG_KICKER に化けた。
    """
    path = tmp_path / "log.jsonl"
    _write(path, [_tracker_kick(9.5, robot=1), _tracker_kick(10.3, robot=3)])
    log = PassPlanLog(str(path))
    kick = log.tracker_kick_near(10.0)
    assert kick is not None
    assert kick["robot_id"] == 1


def test_tracker_kick_prefers_start_timestamp_over_arrival(tmp_path):
    """キック時刻は受信時刻ではなく start_timestamp を使う。"""
    path = tmp_path / "log.jsonl"
    _write(path, [_tracker_kick(10.4, start=9.8)])
    log = PassPlanLog(str(path))
    # 受信は 10.4（窓の外）だが、キックは 9.8 なので拾える。
    kick = log.tracker_kick_near(10.0, window=0.5)
    assert kick is not None
    assert math.isclose(PassPlanLog.kick_time(kick), 9.8)


def test_tracker_kick_absent_is_not_an_error(tmp_path):
    """kicked_ball は optional。来なくても落ちず、None を返すだけ。"""
    path = tmp_path / "log.jsonl"
    _write(path, [_plan(1.0, "PLANNING")])
    log = PassPlanLog(str(path))
    assert log.tracker_kick_near(10.0) is None


# ─── 判定 ────────────────────────────────────────────────────────────────────


def test_verdict_as_planned():
    plan = _plan(1.0, "PLANNING", kicker=1, receiver=2, x=1.0, y=0.0)
    verdict, error = _verdict(_trial(kicker=1, receiver=2, end_pos=(1.2, 0.0)), plan)
    assert verdict == "AS_PLANNED"
    assert math.isclose(error, 0.2, abs_tol=1e-6)


def test_verdict_point_off_beyond_tolerance():
    plan = _plan(1.0, "PLANNING", x=1.0, y=0.0)
    far = (1.0 + RECEIVE_POINT_TOLERANCE + 0.1, 0.0)
    verdict, _ = _verdict(_trial(end_pos=far), plan)
    assert verdict == "POINT_OFF"


def test_verdict_wrong_kicker_takes_priority():
    """受け手がたまたま計画どおりでも、別の味方が蹴ったなら計画どおりではない。"""
    plan = _plan(1.0, "PLANNING", kicker=1, receiver=2)
    verdict, _ = _verdict(_trial(kicker=5, receiver=2), plan)
    assert verdict == "WRONG_KICKER"


def test_verdict_wrong_receiver():
    plan = _plan(1.0, "PLANNING", kicker=1, receiver=2)
    verdict, _ = _verdict(_trial(kicker=1, receiver=7), plan)
    assert verdict == "WRONG_RECEIVER"


def test_verdict_no_plan():
    verdict, error = _verdict(_trial(), None)
    assert verdict == "NO_PLAN"
    assert math.isnan(error)


def test_verdict_passes_through_failed_outcome():
    """パスが成立していない試行は、計画との照合以前の問題として理由をそのまま出す。"""
    verdict, error = _verdict(_trial(outcome="INTERCEPTED"), _plan(1.0, "PLANNING"))
    assert verdict == "INTERCEPTED"
    assert math.isnan(error)
