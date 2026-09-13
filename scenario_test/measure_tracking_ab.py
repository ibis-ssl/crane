#!/usr/bin/env python3
"""位置制御の追従品質を計測する A/B 比較スクリプト（assert しない・JSON を吐く）。

`measure_pass_rate.py` と同じ流儀で、pytest の合否ではなく数値を出す。

## 何を比較するのか

| | Arm A（旧構成） | Arm B（新構成） |
|---|---|---|
| planner | rvo2（ワイヤ mode 3 = 速度指令） | visibility_graph（ワイヤ mode 4 = 位置指令） |
| 経路 | crane -> simulator-cli | crane -> cm4_sim -> simulator-cli |
| 位置ループ | crane 側 | cm4_sim 側 |

主張は「無線経路を位置制御ループの外に出すと、遅延・ジッタ・ロスに強くなる」。
同一の劣化条件で両者を走らせて比較する。

## 指標の定義（重要）

crane が内部で持つ「指令目標位置」は vision からは観測できない。
Arm B ではワイヤ上の mode 4 パケットに target_global_pos が乗るが、
Arm A（mode 3）には乗らないため、**両アームで比較可能な指標は vision のみから
定義できるものに限られる**。したがって次を用いる:

- `time_to_goal`      : FORCE_START から目標（ボール）の許容半径内に入るまでの秒数
- `max_lateral_deviation` : 開始点→目標を結ぶ直線からの横方向最大ずれ [m]
- `overshoot`         : 一度到達した後に目標から再び離れた最大距離 [m]
- `final_distance`    : 計測終了時点の目標までの距離 [m]
- `path_efficiency`   : 直線距離 / 実移動経路長（1.0 に近いほど無駄がない）
- `min_obstacle_distance` : 障害物（blue）との最小中心間距離 [m]

劣化条件（RX_DELAY_MS 等）はこのスクリプトからは設定できない（cm4_sim / compose 側の
責務）。環境変数を読んで JSON に自己記述として記録し、後で突き合わせられるようにする。

## 使い方

    # Arm A（旧構成・既定の compose）
    python3 scenario_test/measure_tracking_ab.py --config baseline --trials 10 \\
        --out /tmp/ab_baseline.json

    # Arm B（新構成・cm4-loop profile で compose を起動しておく）
    python3 scenario_test/measure_tracking_ab.py --config cm4 --trials 10 \\
        --out /tmp/ab_cm4.json

両者の summary を比べる。劣化条件を変えながら回すには
`scripts/scenario_test/run_ab_comparison.sh` を使う。
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
import statistics
import sys
import time

from rcst.communication import Communication

# ─── シナリオ設定 ────────────────────────────────────────────────────────────
# VISIBILITY_OBSTACLE_AVOIDANCE.py と同じ配置を使う（障害物迂回を伴う移動）。
BALL_POS = (3.5, 0.0)
GK_POS = (-5.8, 0.0)
FIELD_ROBOT_START_X = -2.0
OBSTACLES = [(0.0, 0.0), (0.0, 0.5), (0.0, -0.5)]

GOAL_TOLERANCE = 0.3  # 目標到達とみなす距離 [m]
SAMPLE_INTERVAL = 0.05  # 観測周期 [s]
MOTION_SPEED_THRESHOLD = 0.2  # 「動いた」とみなす速度 [m/s]
COLLISION_DISTANCE = 0.15  # 衝突とみなす中心間距離 [m]（ロボット半径約 0.09m）


@dataclasses.dataclass
class TrackingTrialResult:
    """1 試行の結果。outcome は以下のいずれか:

    REACHED    目標の許容半径内に到達した
    TIMEOUT    タイムアウトまで到達しなかった
    COLLIDED   障害物と衝突した
    NO_MOTION  一度も動かなかった（crane が動作していない可能性）
    """

    outcome: str = "NO_MOTION"
    robot_id: int = -1
    time_to_goal: float = float("nan")
    final_distance: float = float("nan")
    max_lateral_deviation: float = float("nan")
    overshoot: float = 0.0
    path_length: float = 0.0
    straight_distance: float = 0.0
    path_efficiency: float = float("nan")
    min_obstacle_distance: float = float("nan")
    samples: int = 0
    duration: float = 0.0

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


def lateral_deviation(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    """点 P の、線分 AB を含む直線からの垂直距離。A と B が同一なら P-A 距離。"""
    dx, dy = bx - ax, by - ay
    denom = math.hypot(dx, dy)
    if denom < 1e-9:
        return math.hypot(px - ax, py - ay)
    # 外積の絶対値 / |AB|
    return abs(dx * (ay - py) - (ax - px) * dy) / denom


def setup_world(comm) -> None:
    """VISIBILITY_OBSTACLE_AVOIDANCE.py と同じ配置。"""
    comm.send_empty_world()
    comm.send_ball(*BALL_POS)
    comm.send_yellow_robot(0, GK_POS[0], GK_POS[1], 0.0)
    for i in range(1, 8):
        comm.send_yellow_robot(i, FIELD_ROBOT_START_X, 2.4 - i * 0.6, 0.0)
    for idx, (bx, by) in enumerate(OBSTACLES):
        comm.send_blue_robot(idx, bx, by, 0.0)


def run_trial(comm, timeout_sec: float) -> TrackingTrialResult:
    """1 試行: STOP -> 配置 -> FORCE_START -> 追従を観測 -> STOP"""
    comm.change_referee_command("STOP", 1.0)
    setup_world(comm)
    time.sleep(2.0)  # 配置反映と役割割当の安定待ち
    comm.observer.reset()
    comm.change_referee_command("FORCE_START", 0.5)

    goal_x, goal_y = BALL_POS
    started = time.time()

    tracked_id = None
    start_pos = None
    last_pos = None
    path_length = 0.0
    max_dev = 0.0
    min_obstacle = float("inf")
    closest_approach = float("inf")
    overshoot = 0.0
    time_to_goal = float("nan")
    reached = False
    observed_motion = False
    collided = False
    samples = 0

    while time.time() - started < timeout_sec:
        world = comm.observer.get_world()
        yellow = world.get_yellow_robots()
        blue = world.get_blue_robots()

        if comm.observer.robot_speed().some_yellow_robots_over(MOTION_SPEED_THRESHOLD):
            observed_motion = True

        # 追跡対象は「GK 以外で最も目標に近いロボット」を最初に 1 度だけ決める
        if tracked_id is None:
            best_id, best_d = None, float("inf")
            for rid, r in yellow.items():
                if rid == 0:
                    continue
                d = math.hypot(r.x - goal_x, r.y - goal_y)
                if d < best_d:
                    best_id, best_d = rid, d
            if best_id is None:
                time.sleep(SAMPLE_INTERVAL)
                continue
            tracked_id = best_id
            start_pos = (yellow[tracked_id].x, yellow[tracked_id].y)
            last_pos = start_pos

        robot = yellow.get(tracked_id)
        if robot is None:
            time.sleep(SAMPLE_INTERVAL)
            continue

        samples += 1
        px, py = robot.x, robot.y

        path_length += math.hypot(px - last_pos[0], py - last_pos[1])
        last_pos = (px, py)

        dev = lateral_deviation(px, py, start_pos[0], start_pos[1], goal_x, goal_y)
        max_dev = max(max_dev, dev)

        for b in blue.values():
            d = math.hypot(px - b.x, py - b.y)
            min_obstacle = min(min_obstacle, d)
            if d < COLLISION_DISTANCE:
                collided = True

        dist = math.hypot(px - goal_x, py - goal_y)
        closest_approach = min(closest_approach, dist)

        if not reached and dist <= GOAL_TOLERANCE:
            reached = True
            time_to_goal = time.time() - started
        if reached:
            # 到達後に再び離れた分をオーバーシュートとして記録
            overshoot = max(overshoot, dist - GOAL_TOLERANCE)

        if collided:
            break

        time.sleep(SAMPLE_INTERVAL)

    duration = time.time() - started
    comm.change_referee_command("STOP", 1.0)

    if collided:
        outcome = "COLLIDED"
    elif not observed_motion:
        outcome = "NO_MOTION"
    elif reached:
        outcome = "REACHED"
    else:
        outcome = "TIMEOUT"

    straight = (
        math.hypot(goal_x - start_pos[0], goal_y - start_pos[1])
        if start_pos
        else float("nan")
    )
    return TrackingTrialResult(
        outcome=outcome,
        robot_id=tracked_id if tracked_id is not None else -1,
        time_to_goal=time_to_goal,
        final_distance=closest_approach if samples else float("nan"),
        max_lateral_deviation=max_dev if samples else float("nan"),
        overshoot=overshoot,
        path_length=path_length,
        straight_distance=straight,
        path_efficiency=(straight / path_length)
        if path_length > 1e-6
        else float("nan"),
        min_obstacle_distance=min_obstacle
        if min_obstacle != float("inf")
        else float("nan"),
        samples=samples,
        duration=duration,
    )


def _finite(values):
    return [v for v in values if v is not None and not math.isnan(v)]


def _mean(values):
    vals = _finite(values)
    return statistics.fmean(vals) if vals else None


def _median(values):
    vals = _finite(values)
    return statistics.median(vals) if vals else None


def collect_env_config(args) -> dict:
    """劣化条件は compose 側が握っている。JSON を自己記述にするため環境から拾う。"""
    return {
        "arm": args.config,
        "arm_description": {
            "baseline": "crane(rvo2, wire mode 3) -> simulator-cli 直送。位置ループは crane 側。",
            "cm4": "crane(visibility_graph, wire mode 4) -> cm4_sim -> simulator-cli。位置ループは cm4_sim 側。",
        }[args.config],
        "trials": args.trials,
        "timeout_sec": args.timeout,
        "goal_tolerance_m": GOAL_TOLERANCE,
        "degradation": {
            "rx_delay_ms": os.environ.get("RX_DELAY_MS", "0"),
            "rx_jitter_ms": os.environ.get("RX_JITTER_MS", "0"),
            "rx_loss_rate": os.environ.get("RX_LOSS_RATE", "0.0"),
        },
        "topology": {
            "planner": os.environ.get("PLANNER", "(unset)"),
            "crane_target_port": os.environ.get("CRANE_TARGET_PORT", "(unset)"),
            "simulator_ibis_port": os.environ.get("IBIS_PORT", "(unset)"),
            "feedback_sim_mode": os.environ.get("FEEDBACK_SIM_MODE", "(unset)"),
            "compose_profiles": os.environ.get("COMPOSE_PROFILES", "(unset)"),
        },
        "note": (
            "degradation と topology はこのスクリプトが設定したものではなく、"
            "実行時の環境変数を記録しただけである。compose の設定と一致していることを"
            "呼び出し側（run_ab_comparison.sh）が保証すること。"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--config",
        choices=["baseline", "cm4"],
        required=True,
        help="どちらのアームを計測しているか（JSON のメタ情報にのみ使う）",
    )
    parser.add_argument("--trials", type=int, default=10, help="試行回数 (default: 10)")
    parser.add_argument(
        "--timeout",
        type=float,
        default=20.0,
        help="1 試行のタイムアウト秒 (default: 20)",
    )
    parser.add_argument("--out", default="", help="結果 JSON の出力先")
    args = parser.parse_args()

    config = collect_env_config(args)
    print("=== TRACKING A/B MEASUREMENT ===")
    print(json.dumps(config, indent=2, ensure_ascii=False))
    print()

    comm = Communication()
    results = []
    try:
        for i in range(args.trials):
            result = run_trial(comm, timeout_sec=args.timeout)
            results.append(result.to_dict())
            t_goal = (
                "-" if math.isnan(result.time_to_goal) else f"{result.time_to_goal:.2f}"
            )
            dev = result.max_lateral_deviation
            min_obs = result.min_obstacle_distance
            print(
                f"[{i + 1}/{args.trials}] {result.outcome:<9} robot={result.robot_id} "
                f"t_goal={t_goal} dev={dev:.3f} "
                f"overshoot={result.overshoot:.3f} min_obs={min_obs:.3f}"
            )
    finally:
        comm.close()

    outcomes = {}
    for r in results:
        outcomes[r["outcome"]] = outcomes.get(r["outcome"], 0) + 1

    reached = [r for r in results if r["outcome"] == "REACHED"]
    summary = {
        "trials": args.trials,
        "outcomes": outcomes,
        "reached": len(reached),
        "reach_rate": len(reached) / len(results) if results else 0.0,
        "time_to_goal_mean": _mean([r["time_to_goal"] for r in reached]),
        "time_to_goal_median": _median([r["time_to_goal"] for r in reached]),
        "max_lateral_deviation_mean": _mean(
            [r["max_lateral_deviation"] for r in results]
        ),
        "overshoot_mean": _mean([r["overshoot"] for r in results]),
        "final_distance_mean": _mean([r["final_distance"] for r in results]),
        "path_efficiency_mean": _mean([r["path_efficiency"] for r in results]),
        "min_obstacle_distance_min": (
            min(_finite([r["min_obstacle_distance"] for r in results]), default=None)
        ),
    }

    print("\n=== SUMMARY ===")
    print(json.dumps(summary, indent=2, ensure_ascii=False))

    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(
                {"config": config, "summary": summary, "results": results},
                f,
                indent=2,
                ensure_ascii=False,
            )
        print(f"\n結果を保存しました: {args.out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
