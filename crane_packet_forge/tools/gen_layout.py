#!/usr/bin/env python3
# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""robot_packet.h と ibis_sender_node.cpp から layout.py を生成する。

このツールが存在する理由: 同じバイトオフセット表が既に crane の robot_packet.h、
Orion_CM4 の packet_codec.py、scenario_test/dump_ibis_packets.py に散らばっている。
4 つ目を手書きで増やすと、ヘッダが動いたときに黙ってずれる。

特に注意するのは「レンジ」で、これは enum ではなく serialize 本体の
forward(..., <range>) の実引数にしか書かれていない。名前が theta でも
mode 3 の target_global_velocity_theta は M_PI ではなく 32.767 を使う。
名前から推測すると 10 倍ずれ、しかもどこにもエラーが出ない。

使い方:
    python3 tools/gen_layout.py            # crane_packet_forge/layout.py を書き出す
    python3 tools/gen_layout.py --stdout   # 標準出力に出す（テストの差分比較用）
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG_DIR = HERE.parent
CRANE_ROOT = PKG_DIR.parent

HEADER = CRANE_ROOT / "crane_sender" / "include" / "crane_sender" / "robot_packet.h"
SENDER = CRANE_ROOT / "crane_sender" / "src" / "ibis_sender_node.cpp"
FEEDBACK = (
    CRANE_ROOT
    / "crane_robot_receiver"
    / "include"
    / "crane_robot_receiver"
    / "robot_feedback_protocol.hpp"
)
RECEIVER = CRANE_ROOT / "crane_robot_receiver" / "src" / "robot_receiver_node.cpp"

LICENSE_HEADER = """\
# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""

BANNER = '''"""robot_packet.h から自動生成したパケットレイアウト。手で編集しないこと。

再生成:
    python3 tools/gen_layout.py

生成元:
    crane_sender/include/crane_sender/robot_packet.h
    crane_sender/src/ibis_sender_node.cpp   (CommConfig の framing 定数)
    crane_robot_receiver/include/crane_robot_receiver/robot_feedback_protocol.hpp
    crane_robot_receiver/src/robot_receiver_node.cpp   (multicast の既定値)

test/test_layout_sync.py が生成をやり直してこのファイルと突き合わせるので、
ヘッダが動いたらテストが落ちる。
"""
'''


def _strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//[^\n]*", "", text)
    return text


def _squash(text: str) -> str:
    """clang-format の改行をたたんで 1 行の関数呼び出しに戻す。"""
    return re.sub(r"\s+", " ", text)


def _eval_int(expr: str, known: dict[str, int]) -> int:
    """enum の初期化子を評価する。識別子は既知の enum 値と #define だけ許す。"""
    node = expr.strip()
    return int(eval(node, {"__builtins__": {}}, dict(known)))


def parse_defines(header: str) -> dict[str, int]:
    out: dict[str, int] = {}
    for name, value in re.findall(r"#define\s+(\w+)\s+\(?(-?\d+)\)?", header):
        out[name] = int(value)
    return out


def parse_enum(
    header: str, keyword: str, name: str, known: dict[str, int]
) -> dict[str, int]:
    """`enum <name> { ... }` / `typedef enum { ... } <name>;` を辞書にする。"""
    if keyword == "enum":
        match = re.search(
            r"\benum\s+" + name + r"\s*\{(.*?)\}", header, flags=re.DOTALL
        )
    else:
        match = re.search(
            r"\btypedef\s+enum\s*\{(.*?)\}\s*" + name + r"\s*;", header, flags=re.DOTALL
        )
    if match is None:
        raise SystemExit(f"{name} が {HEADER} に見つからない")

    values: dict[str, int] = {}
    next_value = 0
    for entry in match.group(1).split(","):
        entry = entry.strip()
        if not entry:
            continue
        if "=" in entry:
            key, expr = entry.split("=", 1)
            scope = dict(known)
            scope.update(values)
            next_value = _eval_int(expr, scope)
            key = key.strip()
        else:
            key = entry
        values[key] = next_value
        next_value += 1
    return values


def parse_ranges(header: str) -> dict[str, float]:
    """serialize 本体の forward(&data[X_HIGH], &data[X_LOW], value, range) を読む。

    戻り値は HIGH 側のアドレス名 -> range。mode_args 側は data[0]/data[2] を
    使うので別扱いにする。
    """
    flat = _squash(_strip_comments(header))
    ranges: dict[str, float] = {}

    pattern = (
        r"forward\(\s*&serialized->data\[(\w+)\]\s*,"
        r"\s*&serialized->data\[(\w+)\]\s*,"
        r"\s*[^,]+,\s*([A-Za-z_0-9.]+)\s*\)"
    )
    for high, _low, range_expr in re.findall(pattern, flat):
        ranges[high] = _range_value(range_expr)

    for func, base in (
        ("PolarVelocityModeArgs_serialize", "POLAR"),
        ("PositionTargetModeArgs_serialize", "POSITION_TARGET"),
    ):
        body = re.search(
            re.escape(func) + r"\(.*?\)\s*\{(.*?)\n\}",
            _strip_comments(header),
            flags=re.DOTALL,
        )
        if body is None:
            raise SystemExit(f"{func} の本体が見つからない")
        calls = re.findall(
            r"forward\(\s*&data\[(\d+)\]\s*,\s*&data\[(\d+)\]\s*,\s*args->(\w+)\s*,\s*([A-Za-z_0-9.]+)\s*\)",
            _squash(body.group(1)),
        )
        for high, _low, field, range_expr in calls:
            ranges[f"{base}:{field}"] = _range_value(range_expr)
            ranges[f"{base}:{field}:offset"] = int(high)
    return ranges


def _range_value(expr: str) -> float:
    if expr == "M_PI":
        return math.pi
    return float(expr)


def parse_comm_config(sender: str) -> dict[str, object]:
    out: dict[str, object] = {}
    body = re.search(r"namespace CommConfig\s*\{(.*?)\}", sender, flags=re.DOTALL)
    if body is None:
        raise SystemExit(f"CommConfig が {SENDER} に見つからない")
    text = body.group(1)
    for name, value in re.findall(r"constexpr\s+int\s+(\w+)\s*=\s*(\d+)", text):
        out[name] = int(value)
    for name, value in re.findall(
        r'constexpr\s+const\s+char\s*\*\s*(\w+)\s*=\s*"([^"]+)"', text
    ):
        out[name] = value
    return out


def parse_counter_max(sender: str) -> int:
    """crane の check_counter 巡回上限 (`if (++counter_ > 200)`)。"""
    match = re.search(r"\+\+counter_\s*>\s*(\d+)", sender)
    if match is None:
        raise SystemExit(
            "check_counter の巡回上限が ibis_sender_node.cpp に見つからない"
        )
    return int(match.group(1))


def parse_feedback(feedback_text: str) -> tuple[dict[str, object], dict[str, int]]:
    """robot_feedback_protocol.hpp から 128B フィードバックの定数を採る。"""
    out: dict[str, object] = {}
    clean = _strip_comments(feedback_text)
    for name, value in re.findall(
        r"constexpr\s+(?:size_t|int)\s+(\w+)\s*=\s*(\d+)", clean
    ):
        out[name] = int(value)
    for name, value in re.findall(
        r"constexpr\s+uint8_t\s+(\w+)\s*=\s*(0[xX][0-9a-fA-F]+|\d+)", clean
    ):
        out[name] = int(value, 0)
    for name, value in re.findall(
        r"constexpr\s+float\s+(\w+)\s*=\s*([0-9.]+)f?", clean
    ):
        out[name] = float(value)

    # コメント除去後は末尾の `// namespace offset` が消えるので、入れ子の無い {...} で取る。
    body = re.search(r"namespace offset\s*\{([^{}]*)\}", clean, flags=re.DOTALL)
    if body is None:
        raise SystemExit(f"namespace offset が {FEEDBACK} に見つからない")
    offsets = {
        name: int(value)
        for name, value in re.findall(
            r"constexpr\s+int\s+(\w+)\s*=\s*(\d+)", body.group(1)
        )
    }
    return out, offsets


def parse_receiver_defaults(receiver_text: str) -> dict[str, object]:
    """multicast のアドレス既定値。crane_robot_receiver と同じ規則で受けるため。"""
    out: dict[str, object] = {}
    patterns = {
        "MULTICAST_IP_BASE": r'get_or_declare_parameter\(this,\s*"multicast_ip_base",\s*"([^"]+)"\)',
        "FEEDBACK_PORT_BASE": r'get_or_declare_parameter\(this,\s*"port_base",\s*(\d+)\)',
        "IP_OCTET_OFFSET": r'get_or_declare_parameter\(this,\s*"ip_octet_offset",\s*(\d+)\)',
    }
    for key, pattern in patterns.items():
        match = re.search(pattern, receiver_text)
        if match is None:
            raise SystemExit(f"{key} の既定値が {RECEIVER} に見つからない")
        raw = match.group(1)
        out[key] = raw if not raw.isdigit() else int(raw)
    return out


def render(
    header_text: str, sender_text: str, feedback_text: str, receiver_text: str
) -> str:
    defines = parse_defines(header_text)
    address = parse_enum(header_text, "enum", "Address", defines)
    flags = parse_enum(header_text, "enum", "FlagAddress", defines)
    modes = parse_enum(header_text, "typedef", "ControlMode", defines)
    ranges = parse_ranges(header_text)
    comm = parse_comm_config(sender_text)

    cmd_size = comm["AI_CMD_V2_SIZE"]
    slots = comm["AI_CMD_V2_ROBOT_NUM"]

    lines: list[str] = [LICENSE_HEADER, BANNER, ""]
    lines.append("import math")
    lines.append("")
    lines.append("# --- framing (ibis_sender_node.cpp CommConfig) ---")
    lines.append(f"CMD_SIZE = {cmd_size}")
    lines.append(f"SLOTS = {slots}")
    lines.append("SLOT_SIZE = CMD_SIZE + 1")
    lines.append("PACKET_SIZE = SLOT_SIZE * SLOTS")
    lines.append(f"DEFAULT_PORT = {comm['DEFAULT_PORT']}")
    lines.append(f"BROADCAST_ADDRESS = {comm['BROADCAST_ADDRESS']!r}")
    lines.append(
        f"MAX_ROBOT_ID = SLOTS - 1  # {slots} スロット固定。{slots} 番以上は符号化できない"
    )
    lines.append(f"CHECK_COUNTER_MAX = {parse_counter_max(sender_text)}")
    lines.append(f"MODE_ARGS_SIZE = {defines['MODE_ARGS_SIZE']}")
    lines.append("")
    lines.append("# --- robot_packet.h enum Address ---")
    for name, value in sorted(address.items(), key=lambda kv: kv[1]):
        lines.append(f"{name} = {value}")
    lines.append("")
    lines.append("# --- robot_packet.h enum FlagAddress ---")
    lines.append("FLAG_BITS = {")
    for name, value in sorted(flags.items(), key=lambda kv: kv[1]):
        lines.append(f"    {name!r}: {value},")
    lines.append("}")
    lines.append("")
    lines.append("# --- robot_packet.h ControlMode ---")
    lines.append("CONTROL_MODES = {")
    for name, value in sorted(modes.items(), key=lambda kv: kv[1]):
        lines.append(f"    {name!r}: {value},")
    lines.append("}")
    lines.append("")
    lines.append("# --- serialize 本体の forward(..., range) から採取したレンジ ---")
    lines.append(
        "# 名前から推測してはいけない。mode 3 の theta は M_PI ではなく 32.767。"
    )
    lines.append("RANGES = {")
    for name in sorted(k for k in ranges if not k.endswith(":offset")):
        lines.append(f"    {name!r}: {_fmt_range(ranges[name])},")
    lines.append("}")
    lines.append("")
    lines.append(
        "# --- mode_args union 内のオフセット (CONTROL_MODE_ARGS からの相対) ---"
    )
    lines.append("MODE_ARGS_OFFSETS = {")
    for name in sorted(k for k in ranges if k.endswith(":offset")):
        lines.append(f"    {name[: -len(':offset')]!r}: {ranges[name]},")
    lines.append("}")
    lines.append("")

    fb, fb_offsets = parse_feedback(feedback_text)
    rx = parse_receiver_defaults(receiver_text)
    lines.append("# --- 128B フィードバック (robot_feedback_protocol.hpp) ---")
    lines.append(f"FEEDBACK_SIZE = {fb['PACKET_SIZE']}")
    lines.append(
        f"FEEDBACK_SYNC = (0x{fb['SYNC_0_VALUE']:02X}, 0x{fb['SYNC_1_VALUE']:02X})"
    )
    lines.append(f"FEEDBACK_TX_VALUE_COUNT = {fb['TX_VALUE_COUNT']}")
    lines.append(f"MOTOR_CURRENT_SCALE = {fb['MOTOR_CURRENT_SCALE']}")
    lines.append(f"KICK_STATE_SCALE = {fb['KICK_STATE_SCALE']}")
    lines.append(f"FLOAT_SIZE = {fb['FLOAT_SIZE']}")
    lines.append(
        "# byte 2 は CRC ではない。G474 は定数 10 を書くだけなので検証に使わない"
    )
    lines.append("# (robot_feedback_protocol.hpp のコメントを参照)。判定は sync のみ。")
    lines.append("FEEDBACK_OFFSETS = {")
    for name, value in sorted(fb_offsets.items(), key=lambda kv: (kv[1], kv[0])):
        lines.append(f"    {name!r}: {value},")
    lines.append("}")
    lines.append("")
    lines.append("# --- multicast の既定値 (robot_receiver_node.cpp) ---")
    lines.append(f"MULTICAST_IP_BASE = {rx['MULTICAST_IP_BASE']!r}")
    lines.append(f"FEEDBACK_PORT_BASE = {rx['FEEDBACK_PORT_BASE']}")
    lines.append(f"IP_OCTET_OFFSET = {rx['IP_OCTET_OFFSET']}")
    lines.append("")
    return "\n".join(lines)


def _fmt_range(value: float) -> str:
    return "math.pi" if abs(value - math.pi) < 1e-12 else repr(value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stdout", action="store_true", help="ファイルに書かず標準出力へ"
    )
    parser.add_argument("--header", type=Path, default=HEADER)
    parser.add_argument("--sender", type=Path, default=SENDER)
    parser.add_argument("--feedback", type=Path, default=FEEDBACK)
    parser.add_argument("--receiver", type=Path, default=RECEIVER)
    args = parser.parse_args()

    text = render(
        args.header.read_text(),
        args.sender.read_text(),
        args.feedback.read_text(),
        args.receiver.read_text(),
    )
    if args.stdout:
        sys.stdout.write(text)
    else:
        out = PKG_DIR / "crane_packet_forge" / "layout.py"
        out.write_text(text)
        print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
