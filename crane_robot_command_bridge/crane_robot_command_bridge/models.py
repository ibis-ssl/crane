from pydantic import BaseModel, Field
from typing import Optional


# FastAPI用のPydanticモデル
class LocalPlannerConfigModel(BaseModel):
    disable_collision_avoidance: bool = False
    disable_goal_area_avoidance: bool = False
    disable_placement_avoidance: bool = False
    disable_ball_avoidance: bool = False
    disable_rule_area_avoidance: bool = False
    max_acceleration: float = 4.0
    max_velocity: float = 6.0
    terminal_velocity: float = 0.0
    priority: int = 0


class Pose2DModel(BaseModel):
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class PositionTargetModeModel(BaseModel):
    target_x: float
    target_y: float
    position_tolerance: float = 0.0
    speed_limit_at_target: float = 0.0


class SimpleVelocityTargetModeModel(BaseModel):
    target_vx: float
    target_vy: float
    speed_limit_at_target: float = 0.0


class RobotCommandModel(BaseModel):
    robot_id: int
    local_goalie_enable: bool = False
    enable_ball_centering_control: bool = False
    local_planner_config: LocalPlannerConfigModel
    chip_enable: bool = False
    stop_flag: bool = False
    lift_up_dribbler_flag: bool = False
    kick_power: float = Field(0.0, ge=0.0, le=1.0)
    dribble_power: float = Field(0.0, ge=0.0, le=1.0)
    enable_local_feedback: bool = False
    target_theta: float = 0.0
    omega_limit: float = 100.0
    theta_tolerance: float = 0.0
    latency_ms: float = 0.0
    elapsed_time_ms_since_last_vision: int = 0
    control_mode: str
    skill_name: str = "API server"
    position_target_mode: Optional[PositionTargetModeModel] = None
    simple_velocity_target_mode: Optional[SimpleVelocityTargetModeModel] = None
