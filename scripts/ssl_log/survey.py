"""surveyサブコマンド: 試合概要を表示する."""

from __future__ import annotations

from collections import Counter

from .formatters import fmt_duration, fmt_t
from .models import LogData


def run_survey(data: LogData) -> None:
    info = data.info
    print("\n=== SSL LOG SURVEY ===")
    print(f"ファイル: {info.path}")
    dur = info.duration_sec
    print(f"時間範囲: 0.00s 〜 {dur:.2f}s ({fmt_duration(dur)})")
    print()

    # チーム情報
    print("=== TEAMS ===")
    if data.referee_states:
        last = data.referee_states[-1]
        y = last
        print(
            f"  Yellow: {y.yellow_name or '(不明)':<20} "
            f"score={y.yellow_score}, yellow_cards={y.yellow_yellow_cards}, red_cards={y.yellow_red_cards}"
        )
        print(
            f"  Blue:   {y.blue_name or '(不明)':<20} "
            f"score={y.blue_score}, yellow_cards={y.blue_yellow_cards}, red_cards={y.blue_red_cards}"
        )
    else:
        print(
            "  Referee情報なし（.log.gzにRefereeメッセージが含まれていない可能性があります）"
        )
    print()

    # referee遷移
    print("=== REFEREE TRANSITIONS ===")
    if data.referee_states:
        prev_cmd = None
        prev_stage = None
        for rs in data.referee_states:
            if rs.command != prev_cmd or rs.stage != prev_stage:
                stage_str = (
                    f"({rs.stage})"
                    if rs.stage != prev_stage or prev_stage is None
                    else ""
                )
                if prev_cmd is None:
                    print(f"  {fmt_t(rs.t)}: {rs.command} {stage_str}")
                else:
                    print(f"  {fmt_t(rs.t)}: {prev_cmd} -> {rs.command} {stage_str}")
                prev_cmd = rs.command
                prev_stage = rs.stage
    else:
        print("  データなし")
    print()

    # ボール統計
    print("=== BALL STATISTICS ===")
    if data.ball_timeline:
        speeds = [b.speed for b in data.ball_timeline]
        max_speed = max(speeds)
        mean_speed = sum(speeds) / len(speeds)
        state_counts = Counter(b.state for b in data.ball_timeline)
        total = len(data.ball_timeline)
        kick_count = sum(
            1
            for i in range(1, len(data.ball_timeline))
            if data.ball_timeline[i].has_kicked_ball
            and not data.ball_timeline[i - 1].has_kicked_ball
        )
        print(
            f"  速度 max={max_speed:.2f}m/s, mean={mean_speed:.2f}m/s, samples={total}"
        )
        for state in ("ROLLING", "STOPPED", "FLYING"):
            pct = state_counts[state] / total * 100
            print(f"  {state}: {pct:.1f}%", end="  ")
        print()
        print(f"  キックイベント: {kick_count}件")
    else:
        print("  データなし")
    print()

    # ロボット数
    print("=== ROBOT COUNT (from Tracker) ===")
    yellow_ids = sorted(
        {rid for (team, rid) in data.robot_timeline if team == "yellow"}
    )
    blue_ids = sorted({rid for (team, rid) in data.robot_timeline if team == "blue"})
    y_str = ",".join(str(i) for i in yellow_ids) or "(なし)"
    b_str = ",".join(str(i) for i in blue_ids) or "(なし)"
    print(f"  Yellow: {len(yellow_ids)}ロボット (IDs: {y_str})")
    print(f"  Blue:   {len(blue_ids)}ロボット (IDs: {b_str})")
    print()

    # ゲームイベントサマリー
    print("=== GAME EVENTS SUMMARY ===")
    if data.referee_states:
        all_events: Counter = Counter()
        yellow_goals = 0
        blue_goals = 0
        for rs in data.referee_states:
            for et in rs.game_event_types:
                all_events[et] += 1
                if et in ("GOAL", "POSSIBLE_GOAL", "GOAL_2"):
                    # スコア変化から判断（referee stateの連続比較）
                    pass
        # ゴール数はスコア変化から計算
        for i in range(1, len(data.referee_states)):
            prev = data.referee_states[i - 1]
            cur = data.referee_states[i]
            if cur.yellow_score > prev.yellow_score:
                yellow_goals += cur.yellow_score - prev.yellow_score
            if cur.blue_score > prev.blue_score:
                blue_goals += cur.blue_score - prev.blue_score

        if all_events:
            for event_type, count in all_events.most_common():
                if event_type in ("GOAL", "POSSIBLE_GOAL", "GOAL_2", "INVALID_GOAL"):
                    print(
                        f"  {event_type}: {count}件 (Yellow={yellow_goals}, Blue={blue_goals})"
                    )
                else:
                    print(f"  {event_type}: {count}件")
        else:
            print("  ゲームイベントなし")
    else:
        print("  データなし")
