"""refereeサブコマンド: referee状態遷移を表示する."""

from __future__ import annotations

import json

from .formatters import fmt_t
from .models import LogData, RefereeState


def run_referee(data: LogData, changes_only: bool = False, fmt: str = "text") -> None:
    states = data.referee_states
    if not states:
        print("Referee情報なし")
        return

    if changes_only:
        filtered: list[RefereeState] = []
        prev_cmd = None
        for rs in states:
            if rs.command != prev_cmd:
                filtered.append(rs)
                prev_cmd = rs.command
    else:
        filtered = states

    if fmt == "json":
        output = [
            {
                "t": rs.t,
                "stage": rs.stage,
                "command": rs.command,
                "command_counter": rs.command_counter,
                "yellow_score": rs.yellow_score,
                "blue_score": rs.blue_score,
            }
            for rs in filtered
        ]
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return

    for rs in filtered:
        ge_str = f"  [{', '.join(rs.game_event_types)}]" if rs.game_event_types else ""
        print(
            f"  {fmt_t(rs.t)}: {rs.command:<35} ({rs.stage})"
            f"  Y={rs.yellow_score} B={rs.blue_score}{ge_str}"
        )
