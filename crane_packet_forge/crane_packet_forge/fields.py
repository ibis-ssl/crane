# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""64 バイト指令パケットの意味づけ（フィールド表）。

オフセット・レンジ・フラグビット・制御モード番号は layout.py（robot_packet.h からの
生成物）だけを参照する。ここに数値を書き写さないこと。

ここで手書きするのは、ヘッダには書かれていない情報だけ:
  - 単位とラベル（人間・GUI 向け）
  - スライダーの推奨範囲（ワイヤ上のレンジ ±32.767 はスライダーには広すぎる）
  - 注意書き
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from dataclasses import field as dc_field

from . import layout as L

# --- kind ---
U8 = "u8"  # 生の 1 バイト
U8_SCALED = "u8_scaled"  # 0.0-1.0 を ×20 して 1 バイト（kick / dribble）
U16 = "u16"  # big-endian 2 バイト整数
TWO_BYTE = "two_byte"  # ±range を 0..65535 へ量子化した 2 バイト
FLAG = "flag"  # FLAGS バイトの 1 ビット
ENUM = "enum"  # control_mode


@dataclass(frozen=True)
class FieldDef:
    key: str  # spec / --set で使うキー。これが唯一の綴り
    label: str
    kind: str
    offset: int  # FLAG のときは FLAGS バイトの位置
    group: str
    unit: str = ""
    wire_range: float | None = None  # TWO_BYTE のみ。layout.RANGES 由来
    bit: int | None = None  # FLAG のみ
    ui_min: float | None = None  # スライダーの推奨下限（ワイヤ上限ではない）
    ui_max: float | None = None
    ui_step: float | None = None
    choices: tuple[tuple[str, int], ...] = ()
    mode: str | None = None  # mode_args のフィールドが属する control_mode 名
    note: str = ""
    size: int = dc_field(default=1)

    @property
    def quantization_range(self) -> float:
        """TWO_BYTE のレンジ。kind が違うのに呼ぶのはバグなので落とす。"""
        if self.wire_range is None:
            raise ValueError(f"{self.key} は 2 バイト量子化フィールドではない")
        return self.wire_range

    @property
    def flag_bit(self) -> int:
        """FLAG のビット位置。kind が違うのに呼ぶのはバグなので落とす。"""
        if self.bit is None:
            raise ValueError(f"{self.key} はフラグビットではない")
        return self.bit

    @property
    def resolution(self) -> float | None:
        """1 LSB あたりの値。±32.767 なら 1mm、±π なら約 9.6e-5 rad。"""
        if self.kind != TWO_BYTE or self.wire_range is None:
            return None
        return 2.0 * self.quantization_range / 65535.0

    @property
    def byte_span(self) -> tuple[int, ...]:
        return tuple(range(self.offset, self.offset + self.size))


def _two_byte(key, label, addr_name, group, unit, ui_min, ui_max, ui_step, note=""):
    return FieldDef(
        key=key,
        label=label,
        kind=TWO_BYTE,
        offset=getattr(L, addr_name),
        group=group,
        unit=unit,
        wire_range=L.RANGES[addr_name],
        ui_min=ui_min,
        ui_max=ui_max,
        ui_step=ui_step,
        note=note,
        size=2,
    )


def _mode_arg(
    key, label, mode_key, mode_name, group, unit, ui_min, ui_max, ui_step, note=""
):
    return FieldDef(
        key=key,
        label=label,
        kind=TWO_BYTE,
        offset=L.CONTROL_MODE_ARGS + L.MODE_ARGS_OFFSETS[mode_key],
        group=group,
        unit=unit,
        wire_range=L.RANGES[mode_key],
        ui_min=ui_min,
        ui_max=ui_max,
        ui_step=ui_step,
        mode=mode_name,
        note=note,
        size=2,
    )


_POLAR = "POLAR_VELOCITY_TARGET_MODE"
_POSITION = "POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE"

FIELDS: tuple[FieldDef, ...] = (
    FieldDef(
        "header",
        "header",
        U8,
        L.HEADER,
        "identity",
        note="crane は常に 0x00 を入れる。同期バイトではない。",
        ui_min=0,
        ui_max=255,
        ui_step=1,
    ),
    FieldDef(
        "check_counter",
        "check_counter",
        U8,
        L.CHECK_COUNTER,
        "identity",
        note=f"0..{L.CHECK_COUNTER_MAX} の巡回。250ms 変化しないと G474 は AI 断とみなす。",
        ui_min=0,
        ui_max=255,
        ui_step=1,
    ),
    _two_byte(
        "vision_global_pos.x",
        "vision X",
        "VISION_GLOBAL_X_HIGH",
        "vision",
        "m",
        -6.5,
        6.5,
        0.01,
    ),
    _two_byte(
        "vision_global_pos.y",
        "vision Y",
        "VISION_GLOBAL_Y_HIGH",
        "vision",
        "m",
        -5.0,
        5.0,
        0.01,
    ),
    _two_byte(
        "vision_global_theta",
        "vision θ",
        "VISION_GLOBAL_THETA_HIGH",
        "vision",
        "rad",
        -math.pi,
        math.pi,
        0.01,
    ),
    _two_byte(
        "target_global_theta",
        "target θ",
        "TARGET_GLOBAL_THETA_HIGH",
        "target",
        "rad",
        -math.pi,
        math.pi,
        0.01,
    ),
    _two_byte(
        "target_global_pos.x",
        "target X",
        "TARGET_GLOBAL_POS_X_HIGH",
        "target",
        "m",
        -6.5,
        6.5,
        0.01,
    ),
    _two_byte(
        "target_global_pos.y",
        "target Y",
        "TARGET_GLOBAL_POS_Y_HIGH",
        "target",
        "m",
        -5.0,
        5.0,
        0.01,
    ),
    _two_byte(
        "terminal_velocity",
        "terminal velocity",
        "TERMINAL_VELOCITY_HIGH",
        "target",
        "m/s",
        0.0,
        4.0,
        0.01,
    ),
    FieldDef(
        "kick_power",
        "kick power",
        U8_SCALED,
        L.KICK_POWER,
        "actuator",
        "0-1",
        ui_min=0.0,
        ui_max=1.0,
        ui_step=0.05,
        note="送出バイトは value×20（0..20）。crane は実機で 0.5 にクランプするが、ここでは効かない。",
    ),
    FieldDef(
        "dribble_power",
        "dribble power",
        U8_SCALED,
        L.DRIBBLE_POWER,
        "actuator",
        "0-1",
        ui_min=0.0,
        ui_max=1.0,
        ui_step=0.05,
        note="送出バイトは value×20（0..20）。",
    ),
    _two_byte(
        "acceleration_limit",
        "acceleration limit",
        "ACCELERATION_LIMIT_HIGH",
        "limits",
        "m/s²",
        0.0,
        10.0,
        0.05,
    ),
    _two_byte(
        "linear_velocity_limit",
        "linear velocity limit",
        "LINEAR_VELOCITY_LIMIT_HIGH",
        "limits",
        "m/s",
        0.0,
        8.0,
        0.05,
    ),
    _two_byte(
        "angular_velocity_limit",
        "angular velocity limit",
        "ANGULAR_VELOCITY_LIMIT_HIGH",
        "limits",
        "rad/s",
        0.0,
        20.0,
        0.1,
    ),
    FieldDef(
        "latency_time_ms",
        "latency",
        U16,
        L.LATENCY_TIME_MS_HIGH,
        "timing",
        "ms",
        ui_min=0,
        ui_max=65535,
        ui_step=1,
        size=2,
        note="crane は uint8 へキャストして送るバグがあり 256ms 以上が折り返す。ここは素の uint16。",
    ),
    FieldDef(
        "elapsed_time_ms_since_last_vision",
        "elapsed since vision",
        U16,
        L.ELAPSED_TIME_MS_SINCE_LAST_VISION_HIGH,
        "timing",
        "ms",
        ui_min=0,
        ui_max=65535,
        ui_step=1,
        size=2,
        note="受信側は 500ms 超で出力を止める。",
    ),
    FieldDef(
        "control_mode",
        "control mode",
        ENUM,
        L.CONTROL_MODE,
        "mode",
        choices=tuple(sorted(L.CONTROL_MODES.items(), key=lambda kv: kv[1])),
        ui_min=0,
        ui_max=255,
        ui_step=1,
        note="旧 CM4 + G474 に mode 4 を送ると暴走する。mode 4 は HEAD の CM4 のみ。",
    ),
    _mode_arg(
        "polar.target_global_velocity_r",
        "velocity r",
        "POLAR:target_global_velocity_r",
        _POLAR,
        "mode_args",
        "m/s",
        -4.0,
        4.0,
        0.01,
    ),
    _mode_arg(
        "polar.target_global_velocity_theta",
        "velocity θ",
        "POLAR:target_global_velocity_theta",
        _POLAR,
        "mode_args",
        "rad",
        -math.pi,
        math.pi,
        0.01,
        note="名前は θ だがワイヤ上のレンジは ±π ではなく ±32.767。",
    ),
    _mode_arg(
        "position_target.terminal_velocity_x",
        "terminal velocity X",
        "POSITION_TARGET:terminal_velocity_x",
        _POSITION,
        "mode_args",
        "m/s",
        -4.0,
        4.0,
        0.01,
    ),
    _mode_arg(
        "position_target.terminal_velocity_y",
        "terminal velocity Y",
        "POSITION_TARGET:terminal_velocity_y",
        _POSITION,
        "mode_args",
        "m/s",
        -4.0,
        4.0,
        0.01,
    ),
)


def _flag_fields() -> tuple[FieldDef, ...]:
    """FLAGS バイトの 8 ビットを全て独立に扱う。

    robot_packet.h が名前を付けているのは bit 0/1/3 だけで、C++ の serialize からは
    残りのビットに触れない。ここでは未定義ビットも個別に立てられるようにする
    （「あり得ない組み合わせ」を作れることがこのツールの目的）。
    """
    named = {bit: name for name, bit in L.FLAG_BITS.items()}
    out = []
    for bit in range(8):
        name = named.get(bit)
        key = f"flags.{name.lower()}" if name else f"flags.bit{bit}"
        label = name.lower() if name else f"bit {bit} (未定義)"
        note = "" if name else "robot_packet.h に定義が無いビット。受信側の挙動は未知。"
        out.append(
            FieldDef(
                key=key,
                label=label,
                kind=FLAG,
                offset=L.FLAGS,
                group="flags",
                bit=bit,
                note=note,
            )
        )
    return tuple(out)


FLAG_FIELDS: tuple[FieldDef, ...] = _flag_fields()
ALL_FIELDS: tuple[FieldDef, ...] = FIELDS + FLAG_FIELDS
BY_KEY: dict[str, FieldDef] = {f.key: f for f in ALL_FIELDS}

MODE_ARGS_SIZE = L.MODE_ARGS_SIZE

GROUP_LABELS = {
    "identity": "識別・カウンタ",
    "vision": "vision 位置",
    "target": "目標",
    "actuator": "キック・ドリブル",
    "limits": "リミット",
    "timing": "タイミング",
    "flags": "フラグ (byte 22)",
    "mode": "制御モード",
    "mode_args": "モード引数 (byte 24-31 union)",
}


def owner_of_byte(index: int) -> FieldDef | None:
    """hex グリッドの色分け用。union は最初に見つかったものを返す。

    FLAGS バイトは 8 ビットが別々のフィールドなので、代表として最初のビットを返す。
    ここで None を返すと GUI が「未使用」と表示してしまう。
    """
    if index == L.FLAGS:
        return FLAG_FIELDS[0]
    for f in ALL_FIELDS:
        if f.kind is FLAG:
            continue
        if index in f.byte_span:
            return f
    return None


def describe_byte(index: int) -> str:
    """hex グリッドの説明文。未使用の理由まで書き分ける。"""
    if index == L.FLAGS:
        return f"byte {index} — flags（8 ビットを個別に指定する）"
    owner = owner_of_byte(index)
    if owner is not None:
        return f"byte {index} — {owner.key}"
    args_end = L.CONTROL_MODE_ARGS + MODE_ARGS_SIZE
    if L.CONTROL_MODE_ARGS <= index < args_end:
        return (
            f"byte {index} — mode_args 領域だが、既知のモードはどちらも"
            "先頭 4 バイトしか使わない"
        )
    return f"byte {index} — crane が書かない未使用領域。受信側の解釈は未定義"
