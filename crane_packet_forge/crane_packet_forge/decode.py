# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""64 バイト指令 → フィールド表。--dry-run / GUI のエコー / decode サブコマンド用。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from . import codec
from . import fields as F
from . import layout as L


@dataclass(frozen=True)
class DecodedField:
    key: str
    label: str
    group: str
    value: Any
    raw: int  # ワイヤ上の生の整数（2 バイトなら uint16、フラグなら 0/1）
    unit: str
    active: bool  # union のうち control_mode と一致する側か
    note: str


def decode_command(data: bytes) -> list[DecodedField]:
    if len(data) != L.CMD_SIZE:
        raise ValueError(f"指令は {L.CMD_SIZE} バイトでなければならない: {len(data)}")

    mode_byte = data[L.CONTROL_MODE]
    active_mode = next(
        (name for name, v in L.CONTROL_MODES.items() if v == mode_byte), None
    )

    out: list[DecodedField] = []
    for definition in F.ALL_FIELDS:
        offset = definition.offset
        kind = definition.kind
        if kind is F.FLAG:
            raw = (data[offset] >> definition.flag_bit) & 0x01
            value: Any = bool(raw)
        elif kind is F.TWO_BYTE:
            raw = codec.decode_u16(data, offset)
            value = codec.decode_two_byte(data, offset, definition.quantization_range)
        elif kind is F.U16:
            raw = codec.decode_u16(data, offset)
            value = raw
        elif kind is F.U8_SCALED:
            raw = data[offset]
            value = codec.decode_power(raw)
        else:
            raw = data[offset]
            value = raw

        out.append(
            DecodedField(
                key=definition.key,
                label=definition.label,
                group=definition.group,
                value=value,
                raw=raw,
                unit=definition.unit,
                # mode_args は control_mode と一致する側だけが受信側に読まれる
                active=definition.mode is None or definition.mode == active_mode,
                note=definition.note,
            )
        )
    return out


def hex_dump(data: bytes, *, width: int = 16) -> str:
    """オフセット付きの hex。CLI と GUI で同じ見え方にする。"""
    lines = []
    for start in range(0, len(data), width):
        chunk = data[start : start + width]
        lines.append(f"{start:02d}: " + " ".join(f"{b:02x}" for b in chunk))
    return "\n".join(lines)


def field_table(decoded: list[DecodedField]) -> str:
    """人間が読む用のフィールド表。--dry-run の主役。"""
    rows = []
    group = None
    for item in decoded:
        if item.group != group:
            group = item.group
            rows.append(f"[{F.GROUP_LABELS.get(group, group)}]")
        mark = " " if item.active else "~"  # ~ は control_mode と一致しない union 側
        if isinstance(item.value, bool):
            shown = "1" if item.value else "0"
        elif isinstance(item.value, float):
            shown = f"{item.value:.4f}"
        else:
            shown = str(item.value)
        unit = f" {item.unit}" if item.unit else ""
        rows.append(
            f" {mark}{item.key:<40s} {shown:>12s}{unit:<6s} raw=0x{item.raw:04x}"
        )
    return "\n".join(rows)


def parse_hex(text: str) -> bytes:
    """ "00 07 ff" / "0007ff" / "0x00,0x07" を受ける。"""
    cleaned = text.replace("0x", " ").replace("0X", " ").replace(",", " ")
    tokens = cleaned.split()
    if len(tokens) == 1 and len(tokens[0]) > 2:
        blob = tokens[0]
        if len(blob) % 2:
            raise ValueError("hex の桁数が奇数")
        return bytes.fromhex(blob)
    return bytes(int(token, 16) for token in tokens)


def split_datagram(data: bytes) -> dict[int, bytes]:
    """715 バイトのデータグラムをスロットごとに分解する。"""
    if len(data) != L.PACKET_SIZE:
        raise ValueError(f"データグラムは {L.PACKET_SIZE} バイト: {len(data)}")
    out = {}
    for slot in range(L.SLOTS):
        base = slot * L.SLOT_SIZE
        out[data[base]] = data[base + 1 : base + 1 + L.CMD_SIZE]
    return out
