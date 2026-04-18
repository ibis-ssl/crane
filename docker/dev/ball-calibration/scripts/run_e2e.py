#!/usr/bin/env python3
"""アルゴリズム別キャリブレーション比較 CLI.

使い方:
  python /app/scripts/run_e2e.py --bag /path/to/rosbag2_xxxx \
      --algorithms linear,huber,ransac,nonlinear_huber
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))


def main():
    parser = argparse.ArgumentParser(description="Ball calibration E2E comparison")
    parser.add_argument(
        "--bag", required=True, help="rosbag ディレクトリまたは mcap ファイルパス"
    )
    parser.add_argument(
        "--algorithms",
        default="linear,huber,ransac,nonlinear_huber",
        help="カンマ区切りのアルゴリズム名",
    )
    parser.add_argument(
        "--bootstrap-n", type=int, default=300, help="ブートストラップ反復数"
    )
    parser.add_argument("--output-yaml", help="YAML 出力先ファイルパス（Huber 結果）")
    args = parser.parse_args()

    from mcap_extractor import extract_trajectories_from_mcap
    from models import OptimizationConfig
    from optimizer import run_optimization
    from yaml_exporter import build_yaml_string

    bag_path = Path(args.bag)
    if bag_path.is_dir():
        mcaps = sorted(bag_path.glob("*.mcap"))
        if not mcaps:
            print(f"ERROR: mcap が見つかりません: {bag_path}", file=sys.stderr)
            sys.exit(1)
        bag_path = mcaps[0]

    print(f"bag: {bag_path}")
    print("軌道抽出中...")
    trajectories = extract_trajectories_from_mcap(bag_path)
    print(f"  抽出軌道数: {len(trajectories)}")

    algorithms = [a.strip() for a in args.algorithms.split(",")]
    best_result = None

    print("\n--- アルゴリズム比較 ---")
    print(
        f"{'algorithm':<20} {'decel':>7} {'RMSE':>7} {'R²':>7} {'n_traj':>7} {'inlier%':>9} {'CI':>20}"
    )
    print("-" * 80)

    for algo in algorithms:
        cfg = OptimizationConfig(
            algorithm=algo,
            aggregation_method="weighted_median",
            bootstrap_n=args.bootstrap_n,
        )
        result = run_optimization(trajectories, cfg)

        if result.success:
            agg = result.aggregate_stats
            ci_str = f"[{agg.ci_decel[0]:.3f}, {agg.ci_decel[1]:.3f}]" if agg else "N/A"
            n_traj = agg.n_trajectories if agg else len(result.kick_data)
            inlier_pct = agg.inlier_trajectory_ratio * 100 if agg else 100.0
            print(
                f"{algo:<20} {result.global_deceleration:>7.4f} {result.global_rmse:>7.4f}"
                f" {result.global_r_squared:>7.4f} {n_traj:>7} {inlier_pct:>8.1f}% {ci_str:>20}"
            )
            if algo == "huber" and best_result is None:
                best_result = result
        else:
            print(f"{algo:<20} {'FAILED':>7}")

    print()

    if args.output_yaml and best_result is not None:
        yaml_text = build_yaml_string(best_result)
        out = Path(args.output_yaml)
        out.write_text(yaml_text)
        print(f"YAML を出力しました: {out}")


if __name__ == "__main__":
    main()
