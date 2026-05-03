#!/usr/bin/env python3
"""SSL公式ログ (.log.gz) 解析CLIツール."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_SCRIPT_DIR = Path(__file__).parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))


def _load_data(log_path: str):
    from ssl_log.reader import load_log

    return load_log(Path(log_path))


def cmd_info(args: argparse.Namespace) -> None:
    data = _load_data(args.log_path)
    info = data.info
    print("\n=== LOG INFO ===")
    print(f"ファイル: {info.path}")
    from ssl_log.formatters import fmt_duration

    print(
        f"時間範囲: 0.00s 〜 {info.duration_sec:.2f}s ({fmt_duration(info.duration_sec)})"
    )
    print(f"フィールド: {info.field_length_m:.1f}m x {info.field_width_m:.1f}m")
    print()
    print("=== TEAMS ===")
    print(f"  Yellow: {info.yellow_team_name or '(不明)'}")
    print(f"  Blue:   {info.blue_team_name or '(不明)'}")
    if data.referee_states:
        last = data.referee_states[-1]
        print(f"  Final score: Yellow={last.yellow_score} Blue={last.blue_score}")
    print()
    print("=== MESSAGE COUNTS ===")
    _MSG_LABELS = {2: "Vision2010", 3: "Referee", 4: "Vision2014", 5: "Tracker2020"}
    for msg_id, count in sorted(info.msg_counts.items()):
        label = _MSG_LABELS.get(msg_id, f"msg_id={msg_id}")
        print(f"  {label}: {count:,}")
    print(f"  Referee遷移(重複除去): {len(data.referee_states)}")
    print(f"  ボールフレーム: {len(data.ball_timeline):,}")
    print(f"  ロボット追跡 (team,id) 種類: {len(data.robot_timeline)}")


def cmd_survey(args: argparse.Namespace) -> None:
    data = _load_data(args.log_path)
    from ssl_log.survey import run_survey

    run_survey(data)


def cmd_track(args: argparse.Namespace) -> None:
    data = _load_data(args.log_path)
    from ssl_log.formatters import parse_time_range
    from ssl_log.tracking import run_track

    time_range = None
    if args.time:
        time_range = parse_time_range(args.time)
    run_track(
        data,
        robot_id=args.robot,
        team=args.team,
        ball=args.ball,
        interval=args.interval,
        time_range=time_range,
        fmt=args.format,
    )


def cmd_events(args: argparse.Namespace) -> None:
    data = _load_data(args.log_path)
    from ssl_log.events import run_events

    run_events(data, types=args.type, fmt=args.format)


def cmd_referee(args: argparse.Namespace) -> None:
    data = _load_data(args.log_path)
    from ssl_log.referee import run_referee

    run_referee(data, changes_only=args.changes_only, fmt=args.format)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="SSL公式ログ (.log.gz) 解析ツール",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
サブコマンド例:
  %(prog)s info match.log.gz
  %(prog)s survey match.log.gz
  %(prog)s referee match.log.gz --changes-only
  %(prog)s events match.log.gz --type goal kick
  %(prog)s track match.log.gz --ball --interval 0.5 --time 100:200
  %(prog)s track match.log.gz --robot 0 --team yellow --interval 1.0
""",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # info
    p = sub.add_parser("info", help="ログ基本情報を表示")
    p.add_argument("log_path")

    # survey
    p = sub.add_parser("survey", help="試合概要を表示（5セクション）")
    p.add_argument("log_path")

    # track
    p = sub.add_parser("track", help="ロボット/ボールの時系列追跡")
    p.add_argument("log_path")
    p.add_argument("--robot", type=int, default=None, help="ロボットID")
    p.add_argument("--team", choices=["blue", "yellow"], default=None, help="チーム")
    p.add_argument("--ball", action="store_true", help="ボールを追跡")
    p.add_argument("--interval", type=float, default=1.0, help="サンプリング間隔 [秒]")
    p.add_argument("--time", type=str, default=None, help="時間範囲 (例: 10.0:200.5)")
    p.add_argument("--format", choices=["text", "json"], default="text")

    # events
    p = sub.add_parser("events", help="ゲームイベントを抽出")
    p.add_argument("log_path")
    p.add_argument(
        "--type",
        nargs="+",
        choices=["kick", "referee", "foul", "goal", "placement", "card"],
        default=None,
        help="表示するイベントタイプ（デフォルト: 全て）",
    )
    p.add_argument("--format", choices=["text", "json"], default="text")

    # referee
    p = sub.add_parser("referee", help="referee状態遷移を表示")
    p.add_argument("log_path")
    p.add_argument("--changes-only", action="store_true", help="コマンド変化のみ表示")
    p.add_argument("--format", choices=["text", "json"], default="text")

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    handlers = {
        "info": cmd_info,
        "survey": cmd_survey,
        "track": cmd_track,
        "events": cmd_events,
        "referee": cmd_referee,
    }
    try:
        handlers[args.command](args)
    except (ValueError, RuntimeError) as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
