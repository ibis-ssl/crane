"""Pydanticデータモデル定義."""

from __future__ import annotations

from pydantic import BaseModel, Field


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


class OptimizationConfig(BaseModel):
    """最適化設定 (C++ OptimizationConfig の移植)."""

    min_trajectory_duration: float = 0.5
    min_data_points_per_trajectory: int = 10
    min_fitting_r_squared: float = 0.6
    velocity_outlier_threshold: float = 2.0
    min_deceleration: float = 0.1
    max_deceleration: float = 2.0


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
