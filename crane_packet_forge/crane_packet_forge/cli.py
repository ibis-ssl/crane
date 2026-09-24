# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""crane-forge — ロボット指令パケットを自由に組み立てて送る CLI。

crane を経由しない。robot_packet.h の 64 バイトを全フィールド・全ビット単位で
組み立て、192.168.20.(100+N):12345 へ 715 バイトのデータグラムとして直接投げる。

安全側の制限は緩和してある（上限で拒否しない）。危険な組み合わせは警告として出す。
--yes-i-know で確認を全部飛ばせる。
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path

from . import fields as F
from . import layout as L
from .assemble import assemble_command
from .decode import decode_command, field_table, hex_dump, parse_hex, split_datagram
from .events import (
    EXIT_INTERRUPTED,
    EXIT_NETWORK,
    EXIT_OK,
    EXIT_RUNTIME,
    EXIT_USAGE,
    EventLog,
)
from .feedback import FeedbackWatcher
from .sender_loop import SenderOptions, SenderSession
from .spec import PacketSpec, SpecError, Step, parse_assignment, parse_raw_assignment

# --- 引数定義 ---


def _add_output_opts(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--json",
        action="store_true",
        dest="json_stdout",
        help="stdout を JSONL にする（エージェント向け）",
    )
    parser.add_argument("--log", help="JSONL の出力先ファイル")
    parser.add_argument("--quiet", action="store_true", help="人間向けの行を出さない")


def _add_spec_opts(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--spec", type=Path, help="packet spec (JSON)。--set より先に読む"
    )
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="フィールドを指定する。キーは spec の fields と同じ綴り",
    )
    parser.add_argument(
        "--raw-byte",
        action="append",
        default=[],
        metavar="INDEX=VALUE",
        help="生バイト上書き。フィールドより後に適用され、常に勝つ",
    )
    parser.add_argument(
        "--base",
        choices=("neutral", "zeros"),
        help="neutral(既定): 未指定の 2 バイト値に 0.0 を符号化する / "
        "zeros: 生ゼロのまま（crane の空きスロットと同じ = -range を意味する）",
    )
    parser.add_argument(
        "--robot", type=int, dest="robot_id", help=f"機体番号 0..{L.MAX_ROBOT_ID}"
    )
    parser.add_argument("--target", help="real（既定） | sim | 任意の IP")
    parser.add_argument(
        "--broadcast",
        action="store_true",
        help=f"{L.BROADCAST_ADDRESS} へ送る。LAN 上の全ロボットに届く",
    )
    parser.add_argument("--rate-hz", type=float, help="既定 62.5（crane と同じ）")
    parser.add_argument(
        "--schedule",
        metavar="KEY=VALUE[,KEY=VALUE]:SECONDS,...",
        help="区間指定。例 'polar.target_global_velocity_r=0:8,"
        "polar.target_global_velocity_r=0.2:2'",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="crane-forge",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    send = sub.add_parser("send", help="パケットを組み立てて送る")
    _add_spec_opts(send)
    _add_output_opts(send)
    send.add_argument(
        "--duration",
        type=float,
        help="秒。省略時は Ctrl-C まで（schedule 指定時は不要）",
    )
    send.add_argument(
        "--interface-ip",
        help="送信元に使う NIC のアドレス。複数 NIC の PC では指定する",
    )
    send.add_argument("--port", type=int, default=L.DEFAULT_PORT)
    send.add_argument(
        "--dry-run",
        action="store_true",
        help="ソケットを開かず hex とフィールド表だけ出す",
    )
    send.add_argument(
        "--freeze-counter",
        action="store_true",
        help="check_counter を巡回させない。250ms 変化しないと受信側は AI 断とみなす",
    )
    send.add_argument(
        "--no-stop-on-exit", action="store_true", help="終了時の停止指令を送らない"
    )
    send.add_argument(
        "--abort-after",
        type=float,
        metavar="SECONDS",
        help="この秒数で停止指令を送らずに打ち切る（crane が落ちた状況の再現）",
    )
    send.add_argument(
        "--watch", action="store_true", help="送信しながらフィードバックも受ける"
    )
    send.add_argument("--yes-i-know", action="store_true", help="警告の確認を飛ばす")
    send.add_argument(
        "--save-spec", type=Path, help="組み立てた spec を JSON で書き出す"
    )

    decode = sub.add_parser("decode", help="64B 指令 / 715B データグラムを読む")
    decode.add_argument("--hex", help='"00 07 ff" か "0007ff"')
    decode.add_argument("--file", type=Path, help="バイナリファイルから読む")
    decode.add_argument(
        "--json", action="store_true", dest="json_stdout", help="JSON で出す"
    )

    watch = sub.add_parser("watch", help="フィードバックを multicast で受ける")
    watch.add_argument("--robot", type=int, dest="robot_id", required=True)
    watch.add_argument("--interface-ip")
    watch.add_argument("--duration", type=float, help="秒。省略時は Ctrl-C まで")
    _add_output_opts(watch)

    serve = sub.add_parser("serve", help="GUI のバックエンドを起動する")
    serve.add_argument("--host", default="0.0.0.0")
    serve.add_argument(
        "--port", type=int, default=int(os.environ.get("HTTP_PORT") or 8094)
    )
    # docker-compose が FORGE_INTERFACE_IP で渡す。複数 NIC のホストで必要
    serve.add_argument(
        "--interface-ip", default=os.environ.get("FORGE_INTERFACE_IP") or None
    )
    serve.add_argument(
        "--web-root", type=Path, help="GUI の静的ファイル（既定はパッケージ同梱）"
    )
    serve.add_argument(
        "--shared-root",
        type=Path,
        help="crane_web_debugger の web/shared。既定は ament から解決",
    )

    sub.add_parser("fields", help="指定できるフィールドの一覧を出す")
    return parser


# --- spec 組み立て ---


def spec_from_args(args: argparse.Namespace) -> PacketSpec:
    spec = PacketSpec.load(args.spec) if args.spec else PacketSpec()

    if args.base is not None:
        spec.base = args.base
    if args.robot_id is not None:
        spec.robot_id = args.robot_id
    if args.target is not None:
        spec.target = args.target
    if args.broadcast:
        spec.broadcast = True
    if args.rate_hz is not None:
        spec.rate_hz = args.rate_hz

    for assignment in args.set:
        key, value = parse_assignment(assignment)
        spec.fields[key] = value
    for assignment in args.raw_byte:
        index, value = parse_raw_assignment(assignment)
        spec.set_raw_byte(index, value)
    if args.schedule:
        spec.steps = parse_schedule(args.schedule)

    spec.validate()
    return spec


def parse_schedule(text: str) -> list[Step]:
    """'key=value[,key=value]:seconds, ...' を Step の列にする。

    区間の区切りは ';'、区間内の複数フィールドは ','。
    例: 'polar.target_global_velocity_r=0:8; polar.target_global_velocity_r=0.2:2'
    """
    steps: list[Step] = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        if ":" not in chunk:
            raise SpecError(
                f"--schedule の区間は 'key=value:seconds' の形式: {chunk!r}"
            )
        assignments, seconds = chunk.rsplit(":", 1)
        try:
            duration = float(seconds)
        except ValueError as exc:
            raise SpecError(f"--schedule の秒数が読めない: {seconds!r}") from exc
        step = Step(duration_s=duration)
        for assignment in assignments.split(","):
            assignment = assignment.strip()
            if not assignment:
                continue
            key, value = parse_assignment(assignment)
            step.fields[key] = value
        steps.append(step)
    if not steps:
        raise SpecError("--schedule が空")
    return steps


# --- サブコマンド ---


def cmd_fields(_args: argparse.Namespace) -> int:
    group = None
    for definition in F.ALL_FIELDS:
        if definition.group != group:
            group = definition.group
            print(f"\n[{F.GROUP_LABELS.get(group, group)}]")
        span = definition.byte_span
        where = f"byte {span[0]}" if len(span) == 1 else f"byte {span[0]}-{span[-1]}"
        if definition.kind is F.FLAG:
            where += f" bit {definition.bit}"
        rng = f" ±{definition.wire_range:g}" if definition.wire_range else ""
        unit = f" [{definition.unit}]" if definition.unit else ""
        print(f"  {definition.key:<42s} {definition.kind:<9s} {where:<18s}{rng}{unit}")
        if definition.note:
            print(f"  {'':<42s} → {definition.note}")
    return EXIT_OK


def cmd_decode(args: argparse.Namespace) -> int:
    if args.hex:
        data = parse_hex(args.hex)
    elif args.file:
        data = args.file.read_bytes()
    else:
        data = parse_hex(sys.stdin.read())

    if len(data) == L.PACKET_SIZE:
        slots = split_datagram(data)
        non_empty = {rid: cmd for rid, cmd in slots.items() if any(cmd)}
        if args.json_stdout:
            print(
                json.dumps(
                    {
                        str(rid): [f.__dict__ for f in decode_command(cmd)]
                        for rid, cmd in non_empty.items()
                    },
                    ensure_ascii=False,
                    default=str,
                )
            )
            return EXIT_OK
        for robot_id, command in non_empty.items():
            print(f"--- slot {robot_id} ---")
            print(hex_dump(command))
            print(field_table(decode_command(command)))
        if not non_empty:
            print("全スロットが空（64B ゼロ埋め）")
        return EXIT_OK

    if len(data) != L.CMD_SIZE:
        print(
            f"{L.CMD_SIZE}B の指令でも {L.PACKET_SIZE}B のデータグラムでもない: {len(data)}B",
            file=sys.stderr,
        )
        return EXIT_USAGE

    decoded = decode_command(data)
    if args.json_stdout:
        print(
            json.dumps([f.__dict__ for f in decoded], ensure_ascii=False, default=str)
        )
    else:
        print(hex_dump(data))
        print(field_table(decoded))
    return EXIT_OK


def cmd_send(args: argparse.Namespace) -> int:
    spec = spec_from_args(args)
    log = EventLog(args.log, json_stdout=args.json_stdout, quiet=args.quiet)

    if args.save_spec:
        args.save_spec.write_text(
            json.dumps(spec.to_json(), ensure_ascii=False, indent=2) + "\n"
        )
        log.emit(
            "SPEC_SAVED", f"{args.save_spec} に書き出した", path=str(args.save_spec)
        )

    command = assemble_command(spec, check_counter=spec.fields.get("check_counter", 0))
    if args.dry_run:
        if args.json_stdout:
            print(
                json.dumps(
                    {
                        "spec": spec.to_json(),
                        "address": spec.resolve_address(),
                        "command_hex": command.hex(),
                        "datagram_size": L.PACKET_SIZE,
                        "fields": [f.__dict__ for f in decode_command(command)],
                        "warnings": spec.warnings(),
                    },
                    ensure_ascii=False,
                    default=str,
                )
            )
        else:
            print(
                f"宛先: {spec.resolve_address()}:{args.port}  robot_id={spec.robot_id}  "
                f"rate={spec.rate_hz}Hz  base={spec.base}"
            )
            print(hex_dump(command))
            print(field_table(decode_command(command)))
            for warning in spec.warnings():
                print(f"[警告] {warning}", file=sys.stderr)
        return EXIT_OK

    if not _confirm(spec, log, args.yes_i_know):
        return EXIT_USAGE

    watcher = None
    if args.watch:
        watcher = FeedbackWatcher(
            spec.robot_id,
            interface_ip=args.interface_ip,
            on_event=lambda kind, message, extra: log.emit(kind, message, **extra),
        )
        try:
            watcher.start()
        except OSError as exc:
            log.emit("ERROR", f"フィードバックの購読に失敗: {exc}")
            return EXIT_NETWORK

    options = SenderOptions(
        interface_ip=args.interface_ip,
        duration_s=args.duration,
        freeze_counter=args.freeze_counter,
        stop_on_exit=not args.no_stop_on_exit,
        abort_after_s=args.abort_after,
        port=args.port,
    )
    session = SenderSession(spec, log, options)

    def _handle_signal(_signum: int, _frame: object) -> None:
        session.stop("interrupt")

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        session.start()
    except OSError as exc:
        log.emit("ERROR", f"送信ソケットを開けない: {exc}")
        return EXIT_NETWORK

    try:
        while session.stats.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        session.stop("interrupt")
        return EXIT_INTERRUPTED
    finally:
        session.stop(session.stats.stopped_reason or "done")
        if watcher:
            watcher.stop()
        log.close()
    return EXIT_OK


def cmd_watch(args: argparse.Namespace) -> int:
    log = EventLog(args.log, json_stdout=args.json_stdout, quiet=args.quiet)
    watcher = FeedbackWatcher(
        args.robot_id,
        interface_ip=args.interface_ip,
        on_event=lambda kind, message, extra: log.emit(kind, message, **extra),
    )
    try:
        watcher.start()
    except OSError as exc:
        log.emit(
            "ERROR", f"購読に失敗: {exc}。--interface-ip でロボット用 NIC を指定する"
        )
        return EXIT_NETWORK

    deadline = time.time() + args.duration if args.duration else None
    try:
        while deadline is None or time.time() < deadline:
            time.sleep(0.2)
    except KeyboardInterrupt:
        return EXIT_INTERRUPTED
    finally:
        watcher.stop()
        log.emit(
            "WATCH_END",
            f"受信 {watcher.stats.received} / sync 不正 {watcher.stats.bad_sync}",
            received=watcher.stats.received,
            bad_sync=watcher.stats.bad_sync,
        )
        log.close()
    return EXIT_OK


def cmd_serve(args: argparse.Namespace) -> int:
    from .server import run_server  # uvicorn/fastapi は serve のときだけ要る

    return run_server(
        host=args.host,
        port=args.port,
        interface_ip=args.interface_ip,
        web_root=args.web_root,
        shared_root=args.shared_root,
    )


def _confirm(spec: PacketSpec, log: EventLog, skip: bool) -> bool:
    """警告を出す。拒否はしない（安全機構は緩和方針）。"""
    from .transport import find_conflicting_senders

    warnings = list(spec.warnings())
    conflicts = find_conflicting_senders()
    if conflicts:
        warnings.insert(
            0,
            (
                "crane の ibis_sender_node が動いている。2 つの送信元が同じ CM4 へ送ると "
                "check_counter が入り乱れ、測定結果そのものが壊れる: "
                + "; ".join(conflicts)
            ),
        )
    for warning in warnings:
        log.warning(warning)

    if not warnings or skip or not sys.stdin.isatty():
        return True
    answer = input("続けますか? [y/N] ").strip().lower()
    return answer in ("y", "yes")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    handlers = {
        "send": cmd_send,
        "decode": cmd_decode,
        "watch": cmd_watch,
        "serve": cmd_serve,
        "fields": cmd_fields,
    }
    try:
        return handlers[args.cmd](args)
    except SpecError as exc:
        print(f"エラー: {exc}", file=sys.stderr)
        return EXIT_USAGE
    except KeyboardInterrupt:
        return EXIT_INTERRUPTED
    except OSError as exc:
        print(f"ネットワークエラー: {exc}", file=sys.stderr)
        return EXIT_NETWORK
    except Exception as exc:  # noqa: BLE001 - CLI の最後の砦
        print(f"予期しないエラー: {exc}", file=sys.stderr)
        return EXIT_RUNTIME


if __name__ == "__main__":
    raise SystemExit(main())
