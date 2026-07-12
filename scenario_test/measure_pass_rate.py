#!/usr/bin/env python3
"""パス成功率の N 試行計測スクリプト（assert しない・改良前後の A/B 比較用）。

使い方（ER-Force sim + crane が起動済みであること）:
  1. make scenario-test-docker-up      # sim + autoref 起動
  2. crane を起動（scripts/scenario_test/run_test.sh が行う起動と同等。
     例: ros2 launch crane_bringup crane.launch.xml sim:=true ...）
  3. source scenario_test_env/bin/activate
  4. cd scenario_test && python measure_pass_rate.py --trials 20 --out pass_rate_before.json

出力: 試行ごとの結果と集計（成功率・結果内訳）を JSON で保存し、サマリーを表示する。
"""

import argparse
import json
from collections import Counter

from pass_helpers import run_pass_trial, setup_buildup_static, setup_under_mark
from rcst.communication import Communication

SCENARIOS = {
    "buildup": setup_buildup_static,
    "under_mark": setup_under_mark,
}


def main() -> None:
    parser = argparse.ArgumentParser(description="パス成功率の反復計測")
    parser.add_argument("--trials", type=int, default=20, help="試行回数")
    parser.add_argument(
        "--scenario",
        choices=sorted(SCENARIOS.keys()),
        default="buildup",
        help="配置シナリオ",
    )
    parser.add_argument(
        "--timeout", type=float, default=25.0, help="1試行のタイムアウト [s]"
    )
    parser.add_argument(
        "--out", default="", help="結果JSONの出力先（省略時は標準出力のみ）"
    )
    args = parser.parse_args()

    setup_fn = SCENARIOS[args.scenario]
    comm = Communication()
    results = []
    try:
        for i in range(args.trials):
            result = run_pass_trial(comm, setup_fn, timeout_sec=args.timeout)
            results.append(result.to_dict())
            print(f"[{i + 1}/{args.trials}] {result.outcome} ({result.to_dict()})")
    finally:
        comm.close()

    outcome_counts = Counter(r["outcome"] for r in results)
    attempts = sum(1 for r in results if r["outcome"] not in ("NO_KICK",))
    successes = outcome_counts.get("SUCCESS", 0)
    summary = {
        "scenario": args.scenario,
        "trials": args.trials,
        "attempts": attempts,
        "success": successes,
        "success_rate": successes / attempts if attempts > 0 else 0.0,
        "outcomes": dict(outcome_counts),
        "avg_kick_speed": (
            sum(r["kick_speed"] for r in results) / len(results) if results else 0.0
        ),
    }

    print("\n=== PASS RATE SUMMARY ===")
    print(json.dumps(summary, indent=2, ensure_ascii=False))

    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(
                {"summary": summary, "results": results},
                f,
                indent=2,
                ensure_ascii=False,
            )
        print(f"結果を保存しました: {args.out}")


if __name__ == "__main__":
    main()
