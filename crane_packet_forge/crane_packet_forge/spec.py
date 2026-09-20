# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""packet spec — CLI と GUI が共有する唯一の入力形式。

CLI は --spec file.json か --set key=value で受け、GUI は同じ JSON を POST する。
キーは fields.py の FieldDef.key と一字一句同じにする（別名を作らない）。
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from dataclasses import field as dc_field
from pathlib import Path
from typing import Any

from . import fields as F
from . import layout as L

BASE_NEUTRAL = "neutral"
BASE_ZEROS = "zeros"
BASES = (BASE_NEUTRAL, BASE_ZEROS)

TARGET_REAL = "real"
TARGET_SIM = "sim"
SIM_ADDRESS = "127.0.0.1"


class SpecError(ValueError):
    """spec が壊れている。CLI は終了コード 2 で返す。"""


@dataclass
class Step:
    duration_s: float
    fields: dict[str, Any] = dc_field(default_factory=dict)

    def to_json(self) -> dict[str, Any]:
        return {"duration_s": self.duration_s, "fields": dict(self.fields)}


@dataclass
class PacketSpec:
    target: str = TARGET_REAL  # real | sim | <ip>
    robot_id: int = 0
    rate_hz: float = 62.5
    base: str = BASE_NEUTRAL
    broadcast: bool = False
    fields: dict[str, Any] = dc_field(default_factory=dict)
    raw_bytes: dict[int, int] = dc_field(default_factory=dict)
    steps: list[Step] = dc_field(default_factory=list)

    # --- 構築 ---

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> PacketSpec:
        if not isinstance(data, dict):
            raise SpecError("spec はオブジェクトでなければならない")
        unknown = set(data) - {
            "target",
            "robot_id",
            "rate_hz",
            "base",
            "broadcast",
            "fields",
            "raw_bytes",
            "steps",
        }
        if unknown:
            raise SpecError(f"spec に未知のキー: {', '.join(sorted(unknown))}")

        spec = cls(
            target=str(data.get("target", TARGET_REAL)),
            robot_id=int(data.get("robot_id", 0)),
            rate_hz=float(data.get("rate_hz", 62.5)),
            base=str(data.get("base", BASE_NEUTRAL)),
            broadcast=bool(data.get("broadcast", False)),
        )
        for key, value in (data.get("fields") or {}).items():
            spec.set_field(key, value)
        for key, value in (data.get("raw_bytes") or {}).items():
            spec.set_raw_byte(key, value)
        for raw_step in data.get("steps") or []:
            step = Step(duration_s=float(raw_step["duration_s"]))
            for key, value in (raw_step.get("fields") or {}).items():
                step.fields[key] = coerce_field_value(key, value)
            spec.steps.append(step)
        spec.validate()
        return spec

    @classmethod
    def load(cls, path: Path) -> PacketSpec:
        try:
            data = json.loads(Path(path).read_text())
        except json.JSONDecodeError as exc:
            raise SpecError(f"{path}: JSON として読めない: {exc}") from exc
        return cls.from_json(data)

    def to_json(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "target": self.target,
            "robot_id": self.robot_id,
            "rate_hz": self.rate_hz,
            "base": self.base,
        }
        if self.broadcast:
            out["broadcast"] = True
        # 空でもキーを落とさない。受け取る側（GUI）が undefined を掴んで落ちる。
        out["fields"] = dict(self.fields)
        out["raw_bytes"] = {str(k): v for k, v in sorted(self.raw_bytes.items())}
        out["steps"] = [s.to_json() for s in self.steps]
        return out

    # --- 変更 ---

    def set_field(self, key: str, value: Any) -> None:
        self.fields[key] = coerce_field_value(key, value)

    def set_raw_byte(self, index: Any, value: Any) -> None:
        idx = _parse_int(index, "raw_bytes のインデックス")
        val = _parse_int(value, f"raw_bytes[{idx}] の値")
        if not 0 <= idx < L.CMD_SIZE:
            raise SpecError(f"raw_bytes のインデックスは 0..{L.CMD_SIZE - 1}: {idx}")
        if not 0 <= val <= 0xFF:
            raise SpecError(f"raw_bytes[{idx}] は 0..255: {val}")
        self.raw_bytes[idx] = val

    # --- 検証 ---

    def validate(self) -> None:
        if self.base not in BASES:
            raise SpecError(f"base は {' / '.join(BASES)} のどれか: {self.base!r}")
        if not 0 <= self.robot_id <= L.MAX_ROBOT_ID:
            raise SpecError(
                f"robot_id は 0..{L.MAX_ROBOT_ID}（データグラムは {L.SLOTS} スロット固定）: {self.robot_id}"
            )
        if self.rate_hz <= 0:
            raise SpecError(f"rate_hz は正の数: {self.rate_hz}")
        for step in self.steps:
            if step.duration_s <= 0:
                raise SpecError(f"steps の duration_s は正の数: {step.duration_s}")

    def warnings(self) -> list[str]:
        """拒否はしないが伝えるべきこと（安全機構は緩和方針）。"""
        out: list[str] = []
        merged = dict(self.fields)
        for step in self.steps:
            merged.update(step.fields)

        polar = [k for k in merged if k.startswith("polar.")]
        position = [k for k in merged if k.startswith("position_target.")]
        if polar and position:
            out.append(
                "mode_args は byte 24-31 の union。polar.* と position_target.* を"
                "両方指定すると後に書いたほうが勝つ: "
                f"{', '.join(polar + position)}"
            )

        mode = merged.get("control_mode")
        if mode == L.CONTROL_MODES["POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE"]:
            out.append(
                "control_mode 4 (位置指令) は CM4 が HEAD の機体でのみ扱える。"
                "旧 CM4 + G474 へ素通しすると暴走する。"
            )
        if mode is not None and mode not in L.CONTROL_MODES.values():
            out.append(
                f"control_mode {mode} は robot_packet.h に定義が無い"
                f"（既知: {sorted(L.CONTROL_MODES.values())}）。受信側の挙動は未知。"
            )

        if not merged.get("flags.is_vision_available"):
            out.append(
                "flags.is_vision_available が 0。G474 は vision 無効時に出力を止めるので"
                "ホイールは回らない（キープアライブ用途ならこれで正しい）。"
            )
        elapsed = merged.get("elapsed_time_ms_since_last_vision")
        if isinstance(elapsed, int) and elapsed > 500:
            out.append(
                f"elapsed_time_ms_since_last_vision={elapsed} は 500ms 超。受信側は出力を止める。"
            )
        undefined_bits = (
            "flags.bit2",
            "flags.bit4",
            "flags.bit5",
            "flags.bit6",
            "flags.bit7",
        )
        for key, value in merged.items():
            if key in undefined_bits and value:
                out.append(
                    f"{key} は robot_packet.h に定義が無いビット。受信側の挙動は未知。"
                )
        unknown_bytes = sorted(i for i in self.raw_bytes if i >= 38)
        if unknown_bytes:
            out.append(
                f"byte {unknown_bytes} は crane が書かない未使用領域。受信側の解釈は未定義。"
            )
        return out

    # --- 宛先 ---

    def resolve_address(self) -> str:
        if self.broadcast:
            return L.BROADCAST_ADDRESS
        if self.target == TARGET_REAL:
            return robot_ip(self.robot_id)
        if self.target == TARGET_SIM:
            return SIM_ADDRESS
        return self.target


def robot_ip(robot_id: int) -> str:
    """実機の CM4 アドレス。crane_robot_receiver の ping と同じ規則。"""
    return f"192.168.20.{100 + robot_id}"


def coerce_field_value(key: str, value: Any) -> Any:
    """フィールドの型に合わせて値を正規化する。未知のキーはここで弾く。"""
    definition = F.BY_KEY.get(key)
    if definition is None:
        raise SpecError(f"未知のフィールド: {key!r}（--list-fields で一覧）")

    if definition.kind is F.FLAG:
        return _parse_bool(value, key)
    if definition.kind is F.ENUM:
        if isinstance(value, str):
            name = value.strip().upper()
            if name in L.CONTROL_MODES:
                return L.CONTROL_MODES[name]
        parsed = _parse_int(value, key)
        if not 0 <= parsed <= 0xFF:
            raise SpecError(f"{key} は 0..255: {parsed}")
        return parsed
    if definition.kind in (F.U8, F.U16):
        parsed = _parse_int(value, key)
        limit = 0xFF if definition.kind is F.U8 else 0xFFFF
        if not 0 <= parsed <= limit:
            raise SpecError(f"{key} は 0..{limit}: {parsed}")
        return parsed
    return _parse_float(value, key)


def parse_assignment(text: str) -> tuple[str, Any]:
    """--set key=value を (key, 正規化済みの値) にする。"""
    if "=" not in text:
        raise SpecError(f"--set は key=value の形式: {text!r}")
    key, raw = text.split("=", 1)
    return key.strip(), coerce_field_value(key.strip(), raw.strip())


def parse_raw_assignment(text: str) -> tuple[int, int]:
    """--raw-byte 37=0xFF を (index, value) にする。"""
    if "=" not in text:
        raise SpecError(f"--raw-byte は index=value の形式: {text!r}")
    index, value = text.split("=", 1)
    return _parse_int(index.strip(), "raw-byte のインデックス"), _parse_int(
        value.strip(), "raw-byte の値"
    )


def _parse_bool(value: Any, key: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    raise SpecError(f"{key} は真偽値 (1/0/true/false): {value!r}")


def _parse_int(value: Any, key: str) -> int:
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if value != int(value):
            raise SpecError(f"{key} は整数: {value!r}")
        return int(value)
    text = str(value).strip()
    try:
        return int(text, 0)  # 0x / 0b 表記を受ける
    except ValueError as exc:
        raise SpecError(f"{key} は整数として読めない: {value!r}") from exc


def _parse_float(value: Any, key: str) -> float:
    if isinstance(value, bool):
        return float(value)
    if isinstance(value, (int, float)):
        return float(value)
    try:
        return float(str(value).strip())
    except ValueError as exc:
        raise SpecError(f"{key} は数値として読めない: {value!r}") from exc
