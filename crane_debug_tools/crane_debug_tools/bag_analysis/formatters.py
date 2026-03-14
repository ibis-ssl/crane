"""テキスト出力フォーマッタ."""


def format_t(timestamp_ns: int, bag_start_ns: int) -> str:
    """タイムスタンプを `t=12.34` 形式にフォーマット."""
    t = (timestamp_ns - bag_start_ns) / 1e9
    return f"t={t:.2f}"


def format_t_sec(t_sec: float) -> str:
    """相対秒数を `t=12.34` 形式にフォーマット."""
    return f"t={t_sec:.2f}"


def format_pos(x: float, y: float) -> str:
    """位置を `(x.xxx,y.yyy)` 形式にフォーマット."""
    return f"({x:.3f},{y:.3f})"


def format_factors(planning_factors) -> str:
    """planning_factors (list[NamedString]) を `Key=Val; Key=Val` 形式にフォーマット."""
    if not planning_factors:
        return "(empty)"
    return "; ".join(f"{pf.name}={pf.value}" for pf in planning_factors)
