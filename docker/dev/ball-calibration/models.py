"""Pydanticデータモデル定義."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

# アルゴリズム選択
Algorithm = Literal["linear", "huber", "ransac", "nonlinear_huber"]
PhysicsModel = Literal["linear_decay", "exponential_decay"]


class TrajectoryData(BaseModel):
    """ボール軌道データ (C++ TrajectoryData の移植)."""

    event_id: int = 0
    kick_power: float = 0.0
    is_chip_kick: bool = False
    time_points: list[float] = Field(default_factory=list)
    positions_x: list[float] = Field(default_factory=list)
    positions_y: list[float] = Field(default_factory=list)
    velocities: list[float] = Field(default_factory=list)

    @property
    def duration(self) -> float:
        if len(self.time_points) < 2:
            return 0.0
        return self.time_points[-1] - self.time_points[0]

    @property
    def max_velocity(self) -> float:
        if not self.velocities:
            return 0.0
        return max(self.velocities)


class TrajectoryInfo(BaseModel):
    """軌道一覧表示用サマリー."""

    event_id: int
    kick_power: float
    is_chip_kick: bool
    data_points: int
    duration: float
    max_velocity: float


class KickDetectorConfig(BaseModel):
    """キックイベント検出設定."""

    min_kick_speed: float = 0.8
    min_delta_v: float = 0.6
    min_accel: float = 8.0
    pre_stationary_frames: int = 0
    ownership_radius: float = 0.15
    ownership_frames: int = 5
    cooldown_s: float = 0.5
    max_kick_speed: float = 30.0


class OptimizationConfig(BaseModel):
    """最適化設定 (C++ OptimizationConfig の移植)."""

    min_trajectory_duration: float = 0.5
    min_data_points_per_trajectory: int = 10
    min_fitting_r_squared: float = 0.6
    velocity_outlier_threshold: float = 2.0
    min_deceleration: float = 0.1
    max_deceleration: float = 2.0
    min_max_velocity: float = 0.5
    min_inlier_ratio: float = 0.3
    # ロバスト推定オプション
    algorithm: Algorithm = "linear"
    physics_model: PhysicsModel = "linear_decay"
    aggregation_method: Literal["grid", "weighted_median", "mm"] = "weighted_median"
    huber_epsilon: float = 1.35
    ransac_residual_threshold: float | None = None
    bootstrap_n: int = 300


class KickPowerVelocityPair(BaseModel):
    """キックパワー-速度ペア (C++ KickPowerVelocityPair の移植)."""

    event_id: int = 0
    kick_power: float = 0.0
    estimated_initial_velocity: float = 0.0
    is_chip_kick: bool = False
    fitting_r_squared: float = 0.0
    trajectory_duration: float = 0.0
    confidence_lower: float = 0.0
    confidence_upper: float = 0.0
    inlier_ratio: float = 1.0
    method: str = "linear"


class PerTrajectoryFit(BaseModel):
    """軌道単体のロバストフィット結果."""

    event_id: int
    method: str
    v0: float
    deceleration: float
    r_squared: float
    rmse: float
    inlier_ratio: float
    weights: list[float] = Field(default_factory=list)
    residuals: list[float] = Field(default_factory=list)
    ci_v0: tuple[float, float] = (0.0, 0.0)
    ci_decel: tuple[float, float] = (0.0, 0.0)
    rejected: bool = False
    rejection_reason: str | None = None


class RobustAggregateStats(BaseModel):
    """軌道横断の集約統計."""

    method: str
    deceleration: float
    ci_decel: tuple[float, float]
    n_trajectories: int
    inlier_trajectory_ratio: float


class OptimizationResult(BaseModel):
    """最適化結果 (C++ OptimizationResult の移植)."""

    success: bool = False
    global_deceleration: float = 0.0
    global_rmse: float = 0.0
    global_r_squared: float = 0.0
    trajectories_analyzed: int = 0
    trajectories_used: int = 0
    kick_data: list[KickPowerVelocityPair] = Field(default_factory=list)
    power_velocity_summary: dict[str, float] = Field(default_factory=dict)
    per_trajectory_fits: list[PerTrajectoryFit] = Field(default_factory=list)
    aggregate_stats: RobustAggregateStats | None = None


class OptimizeRequest(BaseModel):
    """最適化実行リクエスト."""

    enabled_event_ids: list[int] | None = None
    time_ranges: dict[int, tuple[float, float]] | None = None
    config: OptimizationConfig = Field(default_factory=OptimizationConfig)
    kick_detector: KickDetectorConfig | None = None


class PredictRequest(BaseModel):
    """手動パラメータでの予測計算リクエスト."""

    deceleration: float
    event_ids: list[int] | None = None


class LoadPathRequest(BaseModel):
    """サーバー上のファイルパスからの読み込みリクエスト."""

    path: str


class ManualParamsRequest(BaseModel):
    """手動パラメータ更新リクエスト."""

    deceleration: float | None = None
    kick_power_overrides: dict[str, float] | None = None


class PredictedTrajectory(BaseModel):
    """予測軌道データ (可視化用)."""

    event_id: int
    kick_power: float
    time_points: list[float]
    actual_velocities: list[float]
    predicted_velocities: list[float]
    estimated_v0: float
    r_squared: float
