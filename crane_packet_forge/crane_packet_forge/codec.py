# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""robot_packet.h の符号化プリミティブを Python で再現する。

C 側は float32 で計算してから uint16 へキャスト（切り捨て）する。Python の float は
float64 なので、そのまま書くと境界値で 1 LSB ずれる。各段で float32 へ丸めて
convertFloatToTwoByte と同じ結果を出す。

  uint16 = (uint16_t)(32767.f * (float)(val / range) + 32767.f)
"""

from __future__ import annotations

import struct

_F32 = struct.Struct("<f")


def f32(value: float) -> float:
    """float32 の精度へ丸める（C の float 演算 1 段分）。"""
    return _F32.unpack(_F32.pack(value))[0]


def encode_two_byte(value: float, value_range: float) -> bytes:
    """convertFloatToTwoByte と同じ。range でクランプしてから量子化し、切り捨てる。

    クランプを量子化後の整数に対して行うと範囲外の値で 1 LSB ずれるので、
    C と同じく値の側でクランプする。
    """
    rng = f32(value_range)
    val = f32(value)
    if val > rng:
        val = rng
    elif val < -rng:
        val = -rng
    raw = f32(f32(32767.0) * f32(val / rng))
    raw = f32(raw + f32(32767.0))
    # C の (uint16_t) キャストはゼロ方向への切り捨て。クランプ後は常に非負。
    as_int = int(raw)
    as_int = max(0, min(0xFFFF, as_int))
    return bytes([(as_int >> 8) & 0xFF, as_int & 0xFF])


def decode_two_byte(data: bytes, offset: int, value_range: float) -> float:
    """convertTwoByteToFloat と同じ。"""
    raw = (data[offset] << 8) | data[offset + 1]
    rng = f32(value_range)
    return f32(f32(f32(raw - f32(32767.0)) / f32(32767.0)) * rng)


def encode_u16(value: int) -> bytes:
    """convertUInt16ToTwoByte と同じ（big-endian の素の整数）。"""
    value = int(value) & 0xFFFF
    return bytes([(value >> 8) & 0xFF, value & 0xFF])


def decode_u16(data: bytes, offset: int) -> int:
    return (data[offset] << 8) | data[offset + 1]


def encode_power(value: float) -> int:
    """kick_power / dribble_power。C は `data[X] = command->kick_power * 20;`。

    float → uint8 の暗黙変換で切り捨てられ、範囲外は未定義動作になる。
    ここでは 0..255 に収める（自由に組み立てるツールなので 1.0 超も通す）。
    """
    return max(0, min(255, int(f32(f32(value) * f32(20.0)))))


def decode_power(byte: int) -> float:
    return byte / 20.0
