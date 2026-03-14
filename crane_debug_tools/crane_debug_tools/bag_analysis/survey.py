"""概要サーベイ — analyze-rosbag Step 2 テンプレートの移植."""

import math

from .formatters import format_factors, format_pos, format_t_sec
from .models import BagData


def run_survey(bag_data: BagData, sample_interval: float = 5.0) -> str:
    """Bagデータの概要サーベイを実行してテキストで返す.

    Args:
        bag_data: BagData
        sample_interval: ワールドモデル/ゲーム分析のサンプリング間隔（秒）

    Returns:
        サーベイ結果のテキスト
    """
    lines: list[str] = []

    lines.extend(_play_situations(bag_data))
    lines.append("")
    lines.extend(_role_assignments(bag_data))
    lines.append("")
    lines.extend(_world_model_samples(bag_data, sample_interval))
    lines.append("")
    lines.extend(_unique_planning_factors(bag_data))
    lines.append("")
    lines.extend(_robot_velocity_status(bag_data))
    lines.append("")
    lines.extend(_game_analysis_samples(bag_data, sample_interval))
    lines.append("")
    lines.extend(_rosout_warnings(bag_data))

    return "\n".join(lines)


def _play_situations(bag_data: BagData) -> list[str]:
    lines = ["=== PLAY SITUATIONS ==="]
    for tm in bag_data.get("/play_situation"):
        msg = tm.msg
        lines.append(
            f"  {format_t_sec(tm.t)}: {msg.command.name}, reason={msg.reason_text}"
        )
    return lines


def _role_assignments(bag_data: BagData) -> list[str]:
    lines = ["=== ROLE ASSIGNMENTS (last) ==="]
    selects = bag_data.get("/robot_select_results")
    if selects:
        tm = selects[-1]
        lines.append(f"  {format_t_sec(tm.t)}:")
        for r in tm.msg.results:
            lines.append(f"    {r.name}: selected={list(r.selected_robots)}")
    return lines


def _world_model_samples(bag_data: BagData, interval: float) -> list[str]:
    lines = [f"=== WORLD MODEL (every {interval}s) ==="]
    for tm in bag_data.sample("/world_model", interval):
        msg = tm.msg
        ball = msg.ball_info
        bx, by = ball.position.x, ball.position.y
        bvx, bvy = ball.velocity.x, ball.velocity.y
        bspeed = math.sqrt(bvx**2 + bvy**2)
        lines.append(
            f"  {format_t_sec(tm.t)}: ball={format_pos(bx, by)} speed={bspeed:.2f}m/s"
        )
        for r in msg.robot_info_ours:
            dist = math.sqrt((r.pose.x - bx) ** 2 + (r.pose.y - by) ** 2)
            lines.append(
                f"    our[{r.id}]: {format_pos(r.pose.x, r.pose.y)}"
                f" dist_ball={dist:.2f}m det={r.available_vision}"
            )
    return lines


def _unique_planning_factors(bag_data: BagData) -> list[str]:
    lines = [
        "=== CONTROL_TARGETS: UNIQUE PLANNING_FACTORS (after first play_situation) ==="
    ]
    play_situations = bag_data.get("/play_situation")
    start_t = play_situations[0].t if play_situations else 0.0

    seen_factors: dict = {}
    for tm in bag_data.get("/control_targets"):
        if tm.t < start_t:
            continue
        for rc in tm.msg.robot_commands:
            factors = tuple((pf.name, pf.value) for pf in rc.planning_factors)
            key = (rc.robot_id, factors)
            if key not in seen_factors:
                seen_factors[key] = tm.t
                fs = format_factors(rc.planning_factors)
                lines.append(f"  {format_t_sec(tm.t)} robot={rc.robot_id}: [{fs}]")
    return lines


def _robot_velocity_status(bag_data: BagData, interval: float = 10.0) -> list[str]:
    lines = [f"=== ROBOT VELOCITY STATUS (every {interval}s, robot_commands) ==="]
    for tm in bag_data.sample("/robot_commands", interval):
        zero_robots = []
        moving_robots = []
        for rc in tm.msg.robot_commands:
            pv = rc.polar_velocity_target_mode
            vr = pv[0].target_velocity_r if len(pv) > 0 else 0.0
            if abs(vr) < 0.01 and not rc.stop_flag:
                zero_robots.append(rc.robot_id)
            elif abs(vr) >= 0.01:
                moving_robots.append((rc.robot_id, round(vr, 2)))
        lines.append(
            f"  {format_t_sec(tm.t)}: zero_v={zero_robots}, moving={moving_robots}"
        )
    return lines


def _game_analysis_samples(bag_data: BagData, interval: float) -> list[str]:
    lines = [f"=== GAME ANALYSIS (every {interval}s) ==="]
    for tm in bag_data.sample("/game_analysis", interval):
        msg = tm.msg
        lines.append(
            f"  {format_t_sec(tm.t)}: attacker={msg.recommended_attacker_id}"
            f" (score={msg.attacker_suitability_score:.2f}),"
            f" pass_target={msg.pass_target_id}"
        )
    return lines


def _rosout_warnings(bag_data: BagData) -> list[str]:
    lines = ["=== ROSOUT (WARN/ERROR, deduplicated) ==="]
    seen_logs: set = set()
    for tm in bag_data.get("/rosout"):
        msg = tm.msg
        if msg.level < 30:
            continue
        key = (msg.name, msg.msg[:80])
        if key not in seen_logs:
            seen_logs.add(key)
            lines.append(
                f"  {format_t_sec(tm.t)} [{msg.name}] L{msg.level}: {msg.msg[:200]}"
            )
    return lines
