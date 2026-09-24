# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""spec → 64 バイト指令 → 715 バイトデータグラム。

構造体を埋めて RobotCommandSerializedV2_serialize を呼ぶ方式は採らない。あちらは
flags を 3 つの bool から組み立て（bit 0/1/3 しか到達しない）、mode_args は
control_mode に一致する側しか書かないので、「本来あり得ない組み合わせ」が作れない。
それを作れることがこのツールの目的なので、オフセットとレンジだけ借りて
組み立ては自前で持つ。
"""

from __future__ import annotations

from typing import Any

from . import codec
from . import fields as F
from . import layout as L
from .spec import BASE_NEUTRAL, PacketSpec

# base=neutral の既定値。2 バイトフィールドには 0.0 を「符号化して」入れる。
# 生ゼロのままだと 0x0000 は -range（-32.767 m/s 等）を意味してしまう。
NEUTRAL_DEFAULTS: dict[str, Any] = {}
for _f in F.ALL_FIELDS:
    if _f.kind is F.FLAG:
        NEUTRAL_DEFAULTS[_f.key] = False
    elif _f.kind is F.ENUM:
        NEUTRAL_DEFAULTS[_f.key] = L.CONTROL_MODES["POLAR_VELOCITY_TARGET_MODE"]
    elif _f.kind in (F.U8, F.U16):
        NEUTRAL_DEFAULTS[_f.key] = 0
    else:
        NEUTRAL_DEFAULTS[_f.key] = 0.0
del _f


def write_field(buf: bytearray, key: str, value: Any) -> None:
    """1 フィールドを buf へ書く。union の重なりは呼び出し順で後勝ち。"""
    definition = F.BY_KEY[key]
    kind = definition.kind
    offset = definition.offset

    if kind is F.FLAG:
        mask = 1 << definition.flag_bit
        if value:
            buf[offset] |= mask
        else:
            buf[offset] &= ~mask & 0xFF
    elif kind is F.TWO_BYTE:
        buf[offset : offset + 2] = codec.encode_two_byte(
            value, definition.quantization_range
        )
    elif kind is F.U16:
        buf[offset : offset + 2] = codec.encode_u16(value)
    elif kind is F.U8_SCALED:
        buf[offset] = codec.encode_power(value)
    else:  # U8 / ENUM
        buf[offset] = int(value) & 0xFF


def assemble_command(
    spec: PacketSpec,
    *,
    check_counter: int | None = None,
    overrides: dict[str, Any] | None = None,
) -> bytes:
    """64 バイトの指令を作る。

    適用順は「既定 → spec.fields → step の上書き → check_counter → raw_bytes」。
    raw_bytes を最後にするのは、生バイト上書きが常に勝つようにするため。
    """
    buf = bytearray(L.CMD_SIZE)

    if spec.base == BASE_NEUTRAL:
        for key, value in NEUTRAL_DEFAULTS.items():
            write_field(buf, key, value)

    for key, value in spec.fields.items():
        write_field(buf, key, value)

    for key, value in (overrides or {}).items():
        write_field(buf, key, value)

    if check_counter is not None:
        buf[L.CHECK_COUNTER] = int(check_counter) & 0xFF

    for index, value in spec.raw_bytes.items():
        buf[index] = value & 0xFF

    return bytes(buf)


def build_datagram(command: bytes, robot_id: int) -> bytes:
    """715 バイト。crane と同じく全 11 スロットを常に送り、未使用は 64B ゼロ埋め。

    スロット先頭の 1 バイトはスロット番号で、それが robot_id を兼ねる。
    """
    if len(command) != L.CMD_SIZE:
        raise ValueError(
            f"指令は {L.CMD_SIZE} バイトでなければならない: {len(command)}"
        )
    if not 0 <= robot_id <= L.MAX_ROBOT_ID:
        raise ValueError(f"robot_id は 0..{L.MAX_ROBOT_ID}: {robot_id}")

    out = bytearray()
    empty = bytes(L.CMD_SIZE)
    for slot in range(L.SLOTS):
        out.append(slot)
        out += command if slot == robot_id else empty
    return bytes(out)


def stop_command(spec: PacketSpec, *, check_counter: int | None = None) -> bytes:
    """終了時に投げる停止指令。速度 0 かつ STOP_EMERGENCY。

    spec の宛先・モードはそのままに、動きに関わるフィールドだけ潰す。
    """
    overrides: dict[str, Any] = {
        "flags.stop_emergency": True,
        "polar.target_global_velocity_r": 0.0,
        "polar.target_global_velocity_theta": 0.0,
        "kick_power": 0.0,
        "dribble_power": 0.0,
    }
    return assemble_command(spec, check_counter=check_counter, overrides=overrides)
