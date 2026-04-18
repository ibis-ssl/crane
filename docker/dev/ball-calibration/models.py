"""Pydanticデータモデル定義."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

Algorithm = Literal["linear", "huber", "ransac", "nonlinear_huber"]
PhysicsModel = Literal["linear_decay", "exponential_decay"]


class TrajectoryData(BaseModel):
    """ボール軌道データ."""

    event_id: int = 0
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
    data_points: int
    duration: float
    max_velocity: float


class OptimizationConfig(BaseModel):
    """最適化設定."""

    min_trajectory_duration: float = 0.5
    min_data_points_per_trajectory: int = 10
    min_fitting_r_squared: float = 0.6
    velocity_outlier_threshold: float = 2.0
    min_deceleration: float = 0.1
    max_deceleration: float = 2.0
    min_max_velocity: float = 0.5
    min_inlier_ratio: float = 0.3
    algorithm: Algorithm = "linear"
    physics_model: PhysicsModel = "linear_decay"
    aggregation_method: Literal["grid", "weighted_median", "mm"] = "weighted_median"
    huber_epsilon: float = 1.35
    ransac_residual_threshold: float | None = None
    bootstrap_n: int = 300


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
    """最適化結果."""

    success: bool = False
    global_deceleration: float = 0.0
    global_rmse: float = 0.0
    global_r_squared: float = 0.0
    trajectories_analyzed: int = 0
    trajectories_used: int = 0
    per_trajectory_fits: list[PerTrajectoryFit] = Field(default_factory=list)
    aggregate_stats: RobustAggregateStats | None = None


class OptimizeRequest(BaseModel):
    """最適化実行リクエスト."""

    enabled_event_ids: list[int] | None = None
    time_ranges: dict[int, tuple[float, float]] | None = None
    config: OptimizationConfig = Field(default_factory=OptimizationConfig)


class PredictRequest(BaseModel):
    """手動パラメータでの予測計算リクエスト."""

    deceleration: float
    event_ids: list[int] | None = None


class LoadPathRequest(BaseModel):
    """サーバー上のファイルパスからの読み込みリクエスト."""

    path: str


class AddTrajectoryRequest(BaseModel):
    """タイムライン上の手動選択範囲を軌道として追加するリクエスト."""

    start_time: float
    end_time: float


class ManualParamsRequest(BaseModel):
    """手動パラメータ更新リクエスト."""

    deceleration: float | None = None


class PredictedTrajectory(BaseModel):
    """予測軌道データ (可視化用)."""

    event_id: int
    time_points: list[float]
    actual_velocities: list[float]
    predicted_velocities: list[float]
    estimated_v0: float
    r_squared: float
