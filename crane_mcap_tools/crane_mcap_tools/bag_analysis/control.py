"""control_target分析."""

from dataclasses import dataclass
from typing import Any

from .models import BagData


@dataclass
class ControlSnapshot:
    """制御ターゲットのスナップショット."""

    t: float
    robot_id: int
    planning_factors: list[tuple[str, str]]
    kick_power: float
    dribble_power: float
    target_x: float | None
    target_y: float | None
    velocity_r: float | None
    velocity_theta: float | None
    stop_flag: bool
    planner_name: str


@dataclass
class FactorTransition:
    """planning_factorsの遷移イベント."""

    t: float
    robot_id: int
    old_factors: list[tuple[str, str]]
    new_factors: list[tuple[str, str]]

    @property
    def description(self) -> str:
        old_str = "; ".join(f"{n}={v}" for n, v in self.old_factors) or "(empty)"
        new_str = "; ".join(f"{n}={v}" for n, v in self.new_factors) or "(empty)"
        return f"robot={self.robot_id}: [{old_str}] -> [{new_str}]"


def _extract_snapshot(t: float, rc: Any) -> ControlSnapshot:
    """RobotCommandからControlSnapshotを抽出する."""
    pt = rc.position_target_mode
    pv = rc.polar_velocity_target_mode

    target_x = pt[0].target_x if len(pt) > 0 else None
    target_y = pt[0].target_y if len(pt) > 0 else None
    velocity_r = pv[0].target_velocity_r if len(pv) > 0 else None
    velocity_theta = pv[0].target_velocity_theta if len(pv) > 0 else None

    factors = [(pf.name, pf.value) for pf in rc.planning_factors]

    return ControlSnapshot(
        t=t,
        robot_id=rc.robot_id,
        planning_factors=factors,
        kick_power=rc.kick_power,
        dribble_power=rc.dribble_power,
        target_x=target_x,
        target_y=target_y,
        velocity_r=velocity_r,
        velocity_theta=velocity_theta,
        stop_flag=rc.stop_flag,
        planner_name=getattr(rc, "planner_name", ""),
    )


def analyze_control(
    bag_data: BagData,
    robot_id: int,
    interval: float = 0.1,
    time_range: tuple[float, float] | None = None,
) -> list[ControlSnapshot]:
    """特定ロボットのcontrol_targetを時系列で分析する.

    Args:
        bag_data: BagData
        robot_id: 分析するロボットID
        interval: サンプリング間隔（秒）
        time_range: (start_sec, end_sec) bag開始からの相対秒数

    Returns:
        ControlSnapshotのリスト
    """
    data = bag_data if time_range is None else bag_data.time_slice(*time_range)
    result: list[ControlSnapshot] = []

    for tm in data.sample("/control_targets", interval):
        for rc in tm.msg.robot_commands:
            if rc.robot_id == robot_id:
                result.append(_extract_snapshot(tm.t, rc))
                break

    return result


def detect_factor_transitions(
    bag_data: BagData,
    robot_id: int | None = None,
) -> list[FactorTransition]:
    """planning_factorsの遷移を検出する.

    Args:
        bag_data: BagData
        robot_id: 特定ロボットのみ検出する場合のID。Noneの場合は全ロボット。

    Returns:
        FactorTransitionのリスト
    """
    result: list[FactorTransition] = []
    prev_factors: dict[int, list[tuple[str, str]]] = {}

    for tm in bag_data.get("/control_targets"):
        for rc in tm.msg.robot_commands:
            if robot_id is not None and rc.robot_id != robot_id:
                continue

            current = [(pf.name, pf.value) for pf in rc.planning_factors]
            prev = prev_factors.get(rc.robot_id)

            if prev is not None and current != prev:
                result.append(
                    FactorTransition(
                        t=tm.t,
                        robot_id=rc.robot_id,
                        old_factors=prev,
                        new_factors=current,
                    )
                )

            prev_factors[rc.robot_id] = current

    return result
