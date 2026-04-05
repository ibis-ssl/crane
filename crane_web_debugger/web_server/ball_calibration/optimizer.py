"""ボール物理モデル最適化エンジン (C++ SimpleBallPhysicsOptimizer の移植)."""

from __future__ import annotations

import logging
import math

import numpy as np

from .models import (
    KickPowerVelocityPair,
    OptimizationConfig,
    OptimizationResult,
    PredictedTrajectory,
    TrajectoryData,
)

logger = logging.getLogger(__name__)


def _perform_linear_regression(
    x: list[float], y: list[float]
) -> tuple[float, float, float]:
    """線形回帰: y = slope * x + intercept を計算して (slope, intercept, r_squared) を返す."""
    n = len(x)
    if n < 2 or len(y) != n:
        return 0.0, 0.0, 0.0

    x_arr = np.array(x, dtype=float)
    y_arr = np.array(y, dtype=float)

    mean_x = np.mean(x_arr)
    mean_y = np.mean(y_arr)

    ss_xy = np.sum((x_arr - mean_x) * (y_arr - mean_y))
    ss_xx = np.sum((x_arr - mean_x) ** 2)

    if abs(ss_xx) < 1e-10:
        return 0.0, mean_y, 0.0

    slope = ss_xy / ss_xx
    intercept = mean_y - slope * mean_x

    y_pred = slope * x_arr + intercept
    ss_res = np.sum((y_arr - y_pred) ** 2)
    ss_tot = np.sum((y_arr - mean_y) ** 2)

    r_squared = 1.0 - (ss_res / ss_tot) if ss_tot > 1e-10 else 0.0

    return float(slope), float(intercept), float(r_squared)


def filter_quality_data(
    trajectories: list[TrajectoryData],
    config: OptimizationConfig,
) -> list[TrajectoryData]:
    """データ品質フィルタリング (C++ filterQualityData の移植).

    - ストレートキックのみ（チップキックは除外）
    - 最小データ点数チェック
    - キックパワー範囲チェック (0 < power <= 1)
    - 軌道継続時間チェック
    - 速度妥当性チェック (0 <= v <= 20 m/s)
    - 最大速度チェック (>= 0.5 m/s)
    """
    filtered = []
    for traj in trajectories:
        if traj.is_chip_kick:
            continue
        if len(traj.time_points) < config.min_data_points_per_trajectory:
            continue
        if traj.kick_power <= 0.0 or traj.kick_power > 1.0:
            continue
        if traj.duration < config.min_trajectory_duration:
            continue

        max_vel = 0.0
        valid = True
        for v in traj.velocities:
            if v < 0.0 or v > 20.0:
                valid = False
                break
            max_vel = max(max_vel, v)
        if not valid or max_vel < 0.5:
            continue

        filtered.append(traj)
    return filtered


def optimize_global_deceleration(
    trajectories: list[TrajectoryData],
    config: OptimizationConfig,
) -> tuple[float, float]:
    """グリッドサーチで最適な転がり減速度を探索 (C++ optimizeGlobalDeceleration の移植).

    Returns:
        (best_deceleration, min_rmse)
    """
    # 各軌道のslope値から探索範囲を自動決定 (filter_quality_data 通過済みを前提)
    slopes = []
    for traj in trajectories:
        slope, _, r_sq = _perform_linear_regression(traj.time_points, traj.velocities)
        if r_sq > 0.3:
            slopes.append(-slope)  # slope は負なので -slope が減速度

    if not slopes:
        min_slope, max_slope = 0.3, 1.2
    else:
        grid_step = 0.01
        extended_min = min(slopes) - 0.1
        extended_max = max(slopes) + 0.1
        min_slope = max(
            config.min_deceleration, math.floor(extended_min / grid_step) * grid_step
        )
        max_slope = min(
            config.max_deceleration, math.ceil(extended_max / grid_step) * grid_step
        )

    logger.info("減速度探索範囲: %.2f - %.2f m/s²", min_slope, max_slope)

    best_decel = 0.0
    min_rmse = float("inf")
    grid_step = 0.01
    decel = min_slope

    # 各軌道の時系列をnumpy配列に変換 (ループ外で一度だけ)
    traj_arrays = [
        (np.array(traj.time_points), np.array(traj.velocities)) for traj in trajectories
    ]

    while decel <= max_slope + 1e-9:
        total_rmse = 0.0
        valid_count = 0

        for t_arr, v_arr in traj_arrays:
            # v(t) ≈ v0 - decel*t → v(t) + decel*t ≈ v0 (定数)
            expected = (-decel * t_arr).tolist()
            _, v0, _ = _perform_linear_regression(expected, v_arr.tolist())

            if v0 <= 0.1:
                continue

            # numpy vectorized RMSE
            predicted = v0 - decel * t_arr
            rmse = float(np.sqrt(np.mean((v_arr - predicted) ** 2)))

            if rmse < 2.0:
                total_rmse += rmse
                valid_count += 1

        if valid_count >= 3:
            avg_rmse = total_rmse / valid_count
            if avg_rmse < min_rmse:
                min_rmse = avg_rmse
                best_decel = decel

        decel = round(decel + grid_step, 6)

    if best_decel > 0.0:
        logger.info(
            "グローバル減速度最適化成功: %.4f m/s² (RMSE: %.4f)", best_decel, min_rmse
        )
    else:
        logger.warning("グローバル減速度最適化失敗")

    return best_decel, min_rmse


def estimate_initial_velocity(
    trajectory: TrajectoryData,
    config: OptimizationConfig,
) -> KickPowerVelocityPair:
    """個別軌道の初速度を線形回帰で推定 (C++ estimateInitialVelocity の移植)."""
    pair = KickPowerVelocityPair(
        event_id=trajectory.event_id,
        kick_power=trajectory.kick_power,
        is_chip_kick=trajectory.is_chip_kick,
        trajectory_duration=trajectory.duration,
    )

    if len(trajectory.time_points) < config.min_data_points_per_trajectory:
        return pair

    slope, intercept, r_squared = _perform_linear_regression(
        trajectory.time_points, trajectory.velocities
    )

    pair.estimated_initial_velocity = intercept
    pair.fitting_r_squared = r_squared

    # 95%信頼区間 (1.96σ)
    n = len(trajectory.time_points)
    y_pred = [intercept + slope * t for t in trajectory.time_points]
    velocity_std = math.sqrt(
        sum((v - p) ** 2 for v, p in zip(trajectory.velocities, y_pred)) / n
    )
    margin = 1.96 * velocity_std
    pair.confidence_lower = intercept - margin
    pair.confidence_upper = intercept + margin

    return pair


def generate_kick_power_mapping(
    kick_data: list[KickPowerVelocityPair],
) -> dict[str, float]:
    """パワー別平均速度マッピングを生成 (0.1刻み)."""
    target_powers = [i / 10.0 for i in range(11)]
    mapping = {}

    for target_power in target_powers:
        velocities = [
            k.estimated_initial_velocity
            for k in kick_data
            if not k.is_chip_kick and abs(k.kick_power - target_power) < 0.05
        ]
        key = f"power_{int(target_power * 100):02d}"
        if velocities:
            mapping[key] = sum(velocities) / len(velocities)

    return mapping


def run_optimization(
    trajectories: list[TrajectoryData],
    config: OptimizationConfig,
    enabled_event_ids: list[int] | None = None,
    time_ranges: dict[int, tuple[float, float]] | None = None,
) -> OptimizationResult:
    """最適化パイプライン全体を実行."""
    result = OptimizationResult()
    result.trajectories_analyzed = len(trajectories)

    # 選択軌道でフィルタリング
    if enabled_event_ids is not None:
        enabled_set = set(enabled_event_ids)
        trajectories = [t for t in trajectories if t.event_id in enabled_set]

    # 時間範囲のトリミング
    if time_ranges:
        trimmed = []
        for traj in trajectories:
            if traj.event_id in time_ranges:
                start, end = time_ranges[traj.event_id]
                end = (
                    end if end >= 0 else traj.time_points[-1] if traj.time_points else 0
                )
                indices = [
                    i for i, t in enumerate(traj.time_points) if start <= t <= end
                ]
                if indices:
                    new_traj = TrajectoryData(
                        event_id=traj.event_id,
                        kick_power=traj.kick_power,
                        is_chip_kick=traj.is_chip_kick,
                        time_points=[traj.time_points[i] for i in indices],
                        positions_x=[traj.positions_x[i] for i in indices],
                        positions_y=[traj.positions_y[i] for i in indices],
                        velocities=[traj.velocities[i] for i in indices],
                    )
                    trimmed.append(new_traj)
                else:
                    trimmed.append(traj)
            else:
                trimmed.append(traj)
        trajectories = trimmed

    # 品質フィルタリング
    filtered = filter_quality_data(trajectories, config)
    result.trajectories_used = len(filtered)

    if len(filtered) < 3:
        logger.error("有効な軌道データが不足: %d個 (最低3個必要)", len(filtered))
        return result

    # グローバル減速度最適化
    decel, rmse = optimize_global_deceleration(filtered, config)
    if decel <= 0.0:
        logger.error("グローバル減速度最適化に失敗")
        return result
    result.global_deceleration = decel

    # 個別軌道の初速度推定
    kick_data = []
    total_rmse = 0.0
    for traj in filtered:
        pair = estimate_initial_velocity(traj, config)
        if pair.fitting_r_squared >= config.min_fitting_r_squared:
            kick_data.append(pair)
            total_rmse += 1.0 - pair.fitting_r_squared

    result.kick_data = kick_data
    n = len(kick_data)
    result.global_rmse = total_rmse / n if n > 0 else 1.0
    result.global_r_squared = 1.0 - result.global_rmse
    result.power_velocity_summary = generate_kick_power_mapping(kick_data)
    result.success = n >= 3

    logger.info(
        "最適化完了: decel=%.4f, RMSE=%.4f, キックデータ=%d個",
        result.global_deceleration,
        result.global_rmse,
        n,
    )
    return result


def compute_predicted_trajectories(
    trajectories: list[TrajectoryData],
    deceleration: float,
    config: OptimizationConfig,
) -> list[PredictedTrajectory]:
    """指定した減速度で全軌道の予測速度列を計算 (可視化用)."""
    results = []
    for traj in trajectories:
        if len(traj.time_points) < 2:
            continue

        # v0 を線形回帰で推定
        expected = [-deceleration * t for t in traj.time_points]
        _, v0, _ = _perform_linear_regression(expected, traj.velocities)

        predicted = [max(0.0, v0 - deceleration * t) for t in traj.time_points]

        # R²計算
        mean_v = sum(traj.velocities) / len(traj.velocities)
        ss_tot = sum((v - mean_v) ** 2 for v in traj.velocities)
        ss_res = sum((a - p) ** 2 for a, p in zip(traj.velocities, predicted))
        r_sq = 1.0 - ss_res / ss_tot if ss_tot > 1e-10 else 0.0

        results.append(
            PredictedTrajectory(
                event_id=traj.event_id,
                kick_power=traj.kick_power,
                time_points=traj.time_points,
                actual_velocities=traj.velocities,
                predicted_velocities=predicted,
                estimated_v0=v0,
                r_squared=r_sq,
            )
        )
    return results
