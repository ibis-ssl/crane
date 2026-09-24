#!/usr/bin/env python3
"""PassPlan と役割割当を JSONL へ記録する sidecar プロセス。

なぜ別プロセスなのか
--------------------
シナリオテストの venv (`scenario_test_env`) は `include-system-site-packages = false`
なので、pytest 側から rclpy は import できない。venv を作り直して ROS を見せる案は、
rcst が `protobuf<=3.20` を pin しているのに対し ROS 2 Jazzy が別の protobuf を
持ち込むため衝突しうる。

一方 bag による後追い解析も使えない。`scripts/scenario_test/run_test.sh` は
**pytest が終わってから** crane を停止するので、テスト実行中に bag は閉じていない。

そこで「システム python + ROS 環境で走る別プロセスが JSONL を書き、pytest は素の
ファイル I/O で読む」形にする。venv には一切触らず、pytest 内でリアルタイムに
assert できる。`inject_feedback.py` / `dump_ibis_packets.py` と同じ「ROS 外チャネル」系統。

何を記録するのか
----------------
PassPlan の失敗は**沈黙する**。`isUsablePassPlan()` は独立した複数のゲートの AND で、
どれで落ちてもログが出ない。役割割当の先行予約が発火しなかった場合も無言。
そこで「計画そのもの」に加えて「各ゲートの個別結果」と「実際の役割割当」を残す。

時刻は `time.time()`（壁時計）で打つ。pytest 側のキック検出も同じ壁時計を使うので、
キック時点の計画スナップショットをそのまま突き合わせられる。

可用性ゲートについて
--------------------
`isUsablePassPlan` が使う `available()` は
`(available_vision || available_tracker) && available_hardware` という複合条件で、
`available_hardware` は msg に出ない内部状態。Python で再現すると誤診断の元なので、
**生の可用性フラグをそのまま記録**し、判定の再現は行わない。
"""

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from robocup_ssl_msgs.msg import TrackedFrame

from crane_msgs.msg import PassPlan, PlaySituation, RobotSelectResults, WorldModel

STATE_NAMES = {
    PassPlan.STATE_INACTIVE: "INACTIVE",
    PassPlan.STATE_PLANNING: "PLANNING",
    PassPlan.STATE_KICK_READY: "KICK_READY",
    PassPlan.STATE_BALL_IN_FLIGHT: "BALL_IN_FLIGHT",
    PassPlan.STATE_COMPLETED: "COMPLETED",
    PassPlan.STATE_ABORTED: "ABORTED",
}

# 役割割当のうち、PassPlan の成立に直接関わるもの。
WATCHED_SESSIONS = ("attacker_skill", "pass_receive")
# ボールを記録し始める速度 [m/s]。静止中は書かない（JSONL が膨れる）。
BALL_LOG_MIN_SPEED = 0.3


def _finite(*values) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def _in_box(x: float, y: float, box: tuple) -> bool:
    """box = (min_x, min_y, max_x, max_y)。境界は内側として扱う。"""
    min_x, min_y, max_x, max_y = box
    return min_x <= x <= max_x and min_y <= y <= max_y


class PassPlanRecorder(Node):
    def __init__(self, out_path: str):
        super().__init__("pass_plan_recorder")
        # ノードが生きている間ずっと書き続ける。destroy_node() で閉じる。
        self._out = open(out_path, "a", buffering=1, encoding="utf-8")  # noqa: SIM115
        self._last_plan_key = None
        self._last_assign_key = None

        # /world_model は depth 1 で publish される（world_model_publisher.cpp）。
        # 記録側は最新値だけ見られればよいので合わせる。
        world_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            WorldModel, "/world_model", self._on_world_model, world_qos
        )
        self.create_subscription(
            RobotSelectResults, "/robot_select_results", self._on_select_results, 10
        )
        # Tracker のキック検出。crane 本体は tracked_frame を内部で parse するだけで
        # ROS には出さないので、robocup_ssl_comm の tracker_node を併走させて拾う
        # （起動は pass_plan_log._RecorderProcess）。
        self.create_subscription(
            TrackedFrame, "/tracked_frame", self._on_tracked_frame, 10
        )
        self._last_kick_stamp = None
        self._emit({"kind": "meta", "event": "recorder_started"})

    # ─── 出力 ────────────────────────────────────────────────────────────────

    def _emit(self, record: dict) -> None:
        record["t"] = time.time()
        self._out.write(json.dumps(record, ensure_ascii=False) + "\n")

    # ─── /world_model ────────────────────────────────────────────────────────

    def _on_world_model(self, msg: WorldModel) -> None:
        self._record_ball(msg)
        plan = msg.game_analysis.pass_plan
        point = plan.receive_point

        # 変化したときだけ書く。/world_model は高レートなので毎フレーム書くと
        # JSONL が膨れ、pytest 側の読み取りも重くなる。
        key = (
            int(plan.plan_id),
            int(plan.state),
            int(plan.kicker_id),
            int(plan.receiver_id),
            round(float(point.x), 3),
            round(float(point.y), 3),
            round(float(plan.kick_speed), 3),
            int(msg.play_situation.command.value),
        )
        if key == self._last_plan_key:
            return
        self._last_plan_key = key

        self._emit(
            {
                "kind": "plan",
                "ros_time": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                "plan_id": int(plan.plan_id),
                "state": int(plan.state),
                "state_name": STATE_NAMES.get(
                    int(plan.state), f"UNKNOWN({plan.state})"
                ),
                "kicker_id": int(plan.kicker_id),
                "receiver_id": int(plan.receiver_id),
                "receive_point": {"x": float(point.x), "y": float(point.y)},
                "kick_speed": float(plan.kick_speed),
                "is_chip": bool(plan.is_chip),
                "score": float(plan.score),
                "ball_travel_time": float(plan.ball_travel_time),
                "receiver_travel_time": float(plan.receiver_travel_time),
                "command": int(msg.play_situation.command.value),
                "gates": self._gates(msg, plan),
                "availability": self._availability(msg, plan),
            }
        )

    def _gates(self, msg: WorldModel, plan: PassPlan) -> dict:
        """isUsablePassPlan() のうち、msg だけで厳密に再現できるゲート。

        可用性（available()）は内部状態を含むためここには入れない。
        """
        point = plan.receive_point
        our_side_sign = 1.0 if msg.on_positive_half else -1.0

        # ペナルティエリア（world_model_wrapper.cpp の構築と同じ）
        goal_x = our_side_sign * msg.field_info.x * 0.5
        pa_x, pa_half_y = msg.penalty_area_size.x, msg.penalty_area_size.y / 2.0
        if msg.on_positive_half:
            ours_pa = (goal_x - pa_x, -pa_half_y, goal_x, pa_half_y)
        else:
            ours_pa = (goal_x, -pa_half_y, goal_x + pa_x, pa_half_y)
        theirs_pa = (-ours_pa[2], ours_pa[1], -ours_pa[0], ours_pa[3])

        point_finite = _finite(point.x, point.y)
        return {
            "inplay": int(msg.play_situation.command.value) == PlaySituation.INPLAY,
            "state_ok": int(plan.state)
            in (PassPlan.STATE_PLANNING, PassPlan.STATE_BALL_IN_FLIGHT),
            "ids_ok": plan.kicker_id >= 0
            and plan.receiver_id >= 0
            and plan.kicker_id != plan.receiver_id,
            "not_chip": not plan.is_chip,
            "kick_speed_ok": _finite(plan.kick_speed) and plan.kick_speed > 0.0,
            "score_ok": _finite(plan.score) and plan.score > 0.0,
            "point_finite": point_finite,
            "attacking_half": point_finite and point.x * our_side_sign < 0.0,
            "field_inside": point_finite
            and _in_box(
                point.x,
                point.y,
                (
                    -msg.field_info.x / 2.0,
                    -msg.field_info.y / 2.0,
                    msg.field_info.x / 2.0,
                    msg.field_info.y / 2.0,
                ),
            ),
            "outside_penalty_area": point_finite
            and not (
                _in_box(point.x, point.y, ours_pa)
                or _in_box(point.x, point.y, theirs_pa)
            ),
        }

    def _availability(self, msg: WorldModel, plan: PassPlan) -> dict:
        """出し手・受け手の生の可用性フラグ。available() の再現はしない。"""
        by_id = {int(r.id): r for r in msg.robot_info_ours}
        out = {"goalie_id": int(msg.our_goalie_id)}
        for role, robot_id in (
            ("kicker", plan.kicker_id),
            ("receiver", plan.receiver_id),
        ):
            robot = by_id.get(int(robot_id))
            out[role] = (
                None
                if robot is None
                else {
                    "vision": bool(robot.available_vision),
                    "tracker": bool(robot.available_tracker),
                    "feedback": bool(robot.available_feedback),
                }
            )
        return out

    def _record_ball(self, msg: WorldModel) -> None:
        """動いている間のボール速度を記録する。キック初速の較正に使う。

        pytest 側は vision の位置差分でしか速度を測れず、0.08 秒窓の差分では
        減速度の推定が 0.7 設定に対して 1.3〜2.9 とばらついた。world_model の
        ball_info はこれより素性が良い。tracker がボールを見えている間は
        tracker の推定値がそのまま入り（world_model_data_provider.cpp の
        updateBallInfo）、見えていないときだけ vision 由来になる。

        静止中は書かない（JSONL が膨れる）。しきい値は「転がっている」と
        言える下限に置く。
        """
        ball = msg.ball_info
        speed = math.hypot(float(ball.velocity.x), float(ball.velocity.y))
        if speed < BALL_LOG_MIN_SPEED:
            return
        self._emit(
            {
                "kind": "ball",
                "ros_time": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                "x": round(float(ball.position.x), 4),
                "y": round(float(ball.position.y), 4),
                "speed": round(speed, 4),
                "detected": bool(ball.detected),
                "command": int(msg.play_situation.command.value),
            }
        )

    # ─── /tracked_frame ──────────────────────────────────────────────────────

    def _on_tracked_frame(self, msg: TrackedFrame) -> None:
        """Tracker が検出したキックを記録する。

        pytest 側の自前検出は「ボール速度がしきい値を越えた」「一番近い味方が
        居る」「そのロボットから離れていく」の合成で、しきい値の置き方次第で
        ドリブルの小突きを拾ったり、実際のパスを取り逃したりする。Tracker は
        蹴ったロボットの ID・初速・キック時刻をそのまま持っているので、
        推定を挟まずに済む。

        ただしこの情報を出すかどうかは tracked_frame の生成側（autoref）次第
        なので、来ない可能性を前提に「来たら記録する」だけにして、判定を
        これに依存させない。両方を出力して突き合わせてから乗り換える。
        """
        # has_field の既定値は 255（全ビット立ち）なので、フラグだけでは
        # 「生成側が入れた」ことを意味しない。実値でも守る。
        if not msg.has_field & msg.KICKED_BALL_FIELD_SET:
            return
        kicked = msg.kicked_ball
        stamp = float(kicked.start_timestamp)
        if stamp <= 0.0 or stamp == self._last_kick_stamp:
            return
        self._last_kick_stamp = stamp
        has_robot = bool(kicked.has_field & kicked.ROBOT_ID_FIELD_SET)
        self._emit(
            {
                "kind": "tracker_kick",
                "start_timestamp": stamp,
                "x": round(float(kicked.pos.x), 4),
                "y": round(float(kicked.pos.y), 4),
                "vx": round(float(kicked.vel.x), 4),
                "vy": round(float(kicked.vel.y), 4),
                # z が立っていればチップ。直進パスと区別できる。
                "vz": round(float(kicked.vel.z), 4),
                "speed": round(math.hypot(float(kicked.vel.x), float(kicked.vel.y)), 4),
                "robot_id": int(kicked.robot_id.id) if has_robot else None,
                "team": int(kicked.robot_id.team.value) if has_robot else None,
            }
        )

    # ─── /robot_select_results ───────────────────────────────────────────────

    def _on_select_results(self, msg: RobotSelectResults) -> None:
        """役割割当。先行予約（robot_allocator.cpp）が効いたかを直接答える。"""
        sessions = {r.name: [int(i) for i in r.selected_robots] for r in msg.results}
        key = tuple(sorted((name, tuple(ids)) for name, ids in sessions.items()))
        if key == self._last_assign_key:
            return
        self._last_assign_key = key
        self._emit(
            {
                "kind": "assign",
                "watched": {name: sessions.get(name, []) for name in WATCHED_SESSIONS},
                "sessions": sessions,
            }
        )

    def destroy_node(self) -> bool:
        self._emit({"kind": "meta", "event": "recorder_stopped"})
        self._out.close()
        return super().destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, help="JSONL の出力先")
    args = parser.parse_args()

    rclpy.init()
    node = PassPlanRecorder(args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
