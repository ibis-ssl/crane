"""表示フォーマット共通関数."""

from __future__ import annotations


def fmt_t(t: float) -> str:
    """相対秒をt=xxxxx.xx形式でフォーマット."""
    return f"t={t:8.2f}"


def fmt_duration(sec: float) -> str:
    m = int(sec) // 60
    s = sec - m * 60
    return f"{m}m {s:.0f}s" if m else f"{s:.0f}s"


def fmt_pos(x: float, y: float) -> str:
    return f"({x:6.2f}, {y:6.2f})"


def parse_time_range(s: str) -> tuple[float, float]:
    """'10.0:200.5' -> (10.0, 200.5) に変換."""
    parts = s.split(":")
    if len(parts) != 2:
        raise ValueError(f"--time の書式が不正です: {s!r} (例: '10.0:200.5')")
    return float(parts[0]), float(parts[1])
