"""crane_bag CLIエントリポイント."""

import argparse

from .events import ALL_EVENT_TYPES, detect_events
from .reader import BagReader


def _parse_time_range(arg: str | None) -> tuple[float, float] | None:
    """'10.0:30.0' 形式の時間範囲を解析する."""
    if arg is None:
        return None
    parts = arg.split(":")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(
            f"時間範囲は 'start:end' 形式で指定してください（例: 10.0:30.0）: {arg}"
        )
    return float(parts[0]), float(parts[1])


def cmd_info(args: argparse.Namespace) -> None:
    """Bag情報を表示する."""
    reader = BagReader(args.bag_path)
    info = reader.info()
    print(f"Path: {info.path}")
    print(f"Duration: {info.duration_sec:.2f}s")
    print(f"Topics ({len(info.topic_counts)}):")
    for topic in sorted(info.topic_counts.keys()):
        count = info.topic_counts[topic]
        type_name = info.topic_types.get(topic, "unknown")
        print(f"  {topic:50s} {count:6d} msgs  [{type_name}]")


def cmd_survey(args: argparse.Namespace) -> None:
    """概要サーベイを実行する."""
    from .survey import run_survey

    print(f"Reading {args.bag_path} ...")
    reader = BagReader(args.bag_path)
    bag_data = reader.read()
    print(run_survey(bag_data))


def cmd_track(args: argparse.Namespace) -> None:
    """ロボット/ボール追跡を実行する."""
    from .tracking import track_robot

    time_range = _parse_time_range(getattr(args, "time", None))

    print(f"Reading {args.bag_path} ...")
    reader = BagReader(args.bag_path)
    bag_data = reader.read(time_range=time_range)

    is_ours = not getattr(args, "enemy", False)
    side = "enemy" if not is_ours else "ours"
    states = track_robot(
        bag_data,
        robot_id=args.robot,
        is_ours=is_ours,
        interval=args.interval,
        time_range=None,  # time_range already applied at read
    )

    if not states:
        print(f"robot={args.robot} ({side}) のデータが見つかりません")
        return

    print(f"=== ROBOT TRACKING: robot={args.robot} ({side}) ===")
    print(
        f"{'t':>8}  {'x':>8}  {'y':>8}  {'theta':>8}  {'speed':>8}  {'dist_ball':>10}  det"
    )
    for s in states:
        det = "Y" if s.detected else "N"
        print(
            f"{s.t:8.2f}  {s.x:8.3f}  {s.y:8.3f}  {s.theta:8.3f}"
            f"  {s.speed:8.3f}  {s.dist_to_ball:10.3f}  {det}"
        )


def cmd_events(args: argparse.Namespace) -> None:
    """イベント検出を実行する."""
    types = getattr(args, "type", None) or None

    print(f"Reading {args.bag_path} ...")
    reader = BagReader(args.bag_path)
    bag_data = reader.read()

    events = detect_events(bag_data, types=types)

    if not events:
        print("イベントが検出されませんでした")
        return

    print(f"=== EVENTS ({len(events)}) ===")
    for e in events:
        print(f"  t={e.t:8.2f} [{e.event_type:15s}] {e.description}")


def cmd_control(args: argparse.Namespace) -> None:
    """control_target分析を実行する."""
    from .control import analyze_control, detect_factor_transitions

    time_range = _parse_time_range(getattr(args, "time", None))
    changes_only = getattr(args, "changes_only", False)

    print(f"Reading {args.bag_path} ...")
    reader = BagReader(args.bag_path)
    bag_data = reader.read(time_range=time_range)

    if changes_only:
        transitions = detect_factor_transitions(bag_data, robot_id=args.robot)
        if not transitions:
            print(f"robot={args.robot} の planning_factors 変化が検出されませんでした")
            return
        print(f"=== PLANNING_FACTORS TRANSITIONS: robot={args.robot} ===")
        for tr in transitions:
            print(f"  t={tr.t:.2f}  {tr.description}")
    else:
        snapshots = analyze_control(bag_data, robot_id=args.robot)
        if not snapshots:
            print(f"robot={args.robot} のcontrol_targetデータが見つかりません")
            return
        print(f"=== CONTROL_TARGET: robot={args.robot} ===")
        for s in snapshots:
            factors_str = (
                "; ".join(f"{n}={v}" for n, v in s.planning_factors) or "(empty)"
            )
            pos = (
                f"({s.target_x:.3f},{s.target_y:.3f})"
                if s.target_x is not None
                else "N/A"
            )
            vr = f"{s.velocity_r:.3f}" if s.velocity_r is not None else "N/A"
            print(
                f"  t={s.t:.2f}  kick={s.kick_power:.2f}  drb={s.dribble_power:.2f}"
                f"  pos={pos}  vr={vr}  [{factors_str}]"
            )


def main() -> None:
    """CLIエントリポイント."""
    parser = argparse.ArgumentParser(
        prog="crane_bag",
        description="Crane Rosbag分析CLIツール",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # info コマンド
    p_info = subparsers.add_parser("info", help="Bag情報を表示")
    p_info.add_argument("bag_path", help="rosbagディレクトリのパス")
    p_info.set_defaults(func=cmd_info)

    # survey コマンド
    p_survey = subparsers.add_parser("survey", help="概要サーベイを実行")
    p_survey.add_argument("bag_path", help="rosbagディレクトリのパス")
    p_survey.set_defaults(func=cmd_survey)

    # track コマンド
    p_track = subparsers.add_parser("track", help="ロボット追跡")
    p_track.add_argument("bag_path", help="rosbagディレクトリのパス")
    p_track.add_argument("--robot", type=int, required=True, help="ロボットID")
    p_track.add_argument("--enemy", action="store_true", help="敵チームのロボット")
    p_track.add_argument(
        "--time", type=str, default=None, help="時間範囲 (例: 10.0:30.0)"
    )
    p_track.add_argument(
        "--interval", type=float, default=0.5, help="サンプリング間隔（秒）"
    )
    p_track.set_defaults(func=cmd_track)

    # events コマンド
    p_events = subparsers.add_parser("events", help="イベント検出")
    p_events.add_argument("bag_path", help="rosbagディレクトリのパス")
    p_events.add_argument(
        "--type",
        nargs="+",
        choices=ALL_EVENT_TYPES,
        help=f"検出するイベントタイプ: {ALL_EVENT_TYPES}",
    )
    p_events.set_defaults(func=cmd_events)

    # control コマンド
    p_ctrl = subparsers.add_parser("control", help="control_target分析")
    p_ctrl.add_argument("bag_path", help="rosbagディレクトリのパス")
    p_ctrl.add_argument("--robot", type=int, required=True, help="ロボットID")
    p_ctrl.add_argument(
        "--time", type=str, default=None, help="時間範囲 (例: 10.0:30.0)"
    )
    p_ctrl.add_argument(
        "--changes-only", action="store_true", help="planning_factors変化のみ表示"
    )
    p_ctrl.set_defaults(func=cmd_control)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
