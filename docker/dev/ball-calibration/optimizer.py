"""ボール物理モデル最適化エンジン."""

from __future__ import annotations

import logging
import math

import numpy as np

from models import (
    OptimizationConfig,
    OptimizationResult,
    PerTrajectoryFit,
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
    """データ品質フィルタリング."""
    filtered = []
    for traj in trajectories:
        if len(traj.time_points) < config.min_data_points_per_trajectory:
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
        if not valid or max_vel < config.min_max_velocity:
            continue

        filtered.append(traj)
    return filtered


def optimize_global_deceleration(
    trajectories: list[TrajectoryData],
    config: OptimizationConfig,
) -> tuple[float, float]:
    """グリッドサーチで最適な転がり減速度を探索.

    Returns:
        (best_deceleration, min_rmse)
    """
    slopes = []
    for traj in trajectories:
        slope, _, r_sq = _perform_linear_regression(traj.time_points, traj.velocities)
        if r_sq > 0.3:
            slopes.append(-slope)

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

    traj_arrays = [
        (np.array(traj.time_points), np.array(traj.velocities)) for traj in trajectories
    ]

    while decel <= max_slope + 1e-9:
        total_rmse = 0.0
        valid_count = 0

        for t_arr, v_arr in traj_arrays:
            expected = (-decel * t_arr).tolist()
            _, v0, _ = _perform_linear_regression(expected, v_arr.tolist())

            if v0 <= 0.1:
                continue

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


def _estimate_trajectory_fit_linear(
    traj: TrajectoryData,
    config: OptimizationConfig,
) -> PerTrajectoryFit:
    """線形回帰で軌道の初速・減速度を推定し PerTrajectoryFit を返す."""
    slope, intercept, r_squared = _perform_linear_regression(
        traj.time_points, traj.velocities
    )

    n = len(traj.time_points)
    y_pred = [intercept + slope * t for t in traj.time_points]
    velocity_std = math.sqrt(
        sum((v - p) ** 2 for v, p in zip(traj.velocities, y_pred)) / n
    )
    margin = 1.96 * velocity_std

    rejected = r_squared < config.min_fitting_r_squared or intercept <= 0.0
    rejection_reason = None
    if r_squared < config.min_fitting_r_squared:
        rejection_reason = f"R²不足: {r_squared:.3f} < {config.min_fitting_r_squared}"
    elif intercept <= 0.0:
        rejection_reason = "初速が非正"

    return PerTrajectoryFit(
        event_id=traj.event_id,
        method="linear",
        v0=intercept,
        deceleration=-slope,
        r_squared=r_squared,
        rmse=float(
            np.sqrt(np.mean((np.array(traj.velocities) - np.array(y_pred)) ** 2))
        ),
        inlier_ratio=1.0,
        ci_v0=(intercept - margin, intercept + margin),
        ci_decel=(-slope, -slope),
        rejected=rejected,
        rejection_reason=rejection_reason,
    )


def run_optimization(
    trajectories: list[TrajectoryData],
    config: OptimizationConfig,
    enabled_event_ids: list[int] | None = None,
    time_ranges: dict[int, tuple[float, float]] | None = None,
) -> OptimizationResult:
    """最適化パイプライン全体を実行."""
    result = OptimizationResult()
    result.trajectories_analyzed = len(trajectories)

    if enabled_event_ids is not None:
        enabled_set = set(enabled_event_ids)
        trajectories = [t for t in trajectories if t.event_id in enabled_set]

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

    filtered = filter_quality_data(trajectories, config)
    result.trajectories_used = len(filtered)

    if len(filtered) < 3:
        logger.error("有効な軌道データが不足: %d個 (最低3個必要)", len(filtered))
        return result

    if config.algorithm == "linear":
        decel, rmse = optimize_global_deceleration(filtered, config)
        if decel <= 0.0:
            logger.error("グローバル減速度最適化に失敗")
            return result
        result.global_deceleration = decel

        per_traj: list[PerTrajectoryFit] = []
        total_r2 = 0.0
        total_rmse = 0.0
        n_valid = 0
        for traj in filtered:
            fit = _estimate_trajectory_fit_linear(traj, config)
            per_traj.append(fit)
            if not fit.rejected:
                total_r2 += fit.r_squared
                total_rmse += fit.rmse
                n_valid += 1

        result.per_trajectory_fits = per_traj
        result.global_rmse = total_rmse / n_valid if n_valid > 0 else 1.0
        result.global_r_squared = total_r2 / n_valid if n_valid > 0 else 0.0
        result.success = n_valid >= 3
    else:
        result = _run_robust_optimization(filtered, config, result)

    logger.info(
        "最適化完了: decel=%.4f, RMSE=%.4f, 有効軌道=%d個",
        result.global_deceleration,
        result.global_rmse,
        sum(1 for f in result.per_trajectory_fits if not f.rejected),
    )
    return result


def _run_robust_optimization(
    filtered: list[TrajectoryData],
    config: OptimizationConfig,
    result: OptimizationResult,
) -> OptimizationResult:
    """ロバスト推定パイプライン（Huber/RANSAC/非線形 Huber）."""
    from aggregate import aggregate_deceleration
    from robust_fit import FitResult, bootstrap_ci, pick_fit_fn

    fit_fn = pick_fit_fn(
        config.algorithm,
        config.physics_model,
        epsilon=config.huber_epsilon,
        residual_threshold=config.ransac_residual_threshold,
    )

    fits: list[FitResult] = []
    per_traj: list[PerTrajectoryFit] = []
    n_points_list: list[int] = []

    for traj in filtered:
        t_arr = np.array(traj.time_points)
        v_arr = np.array(traj.velocities)
        fit = fit_fn(t_arr, v_arr)

        rejection_reason = fit.rejection_reason
        rejected = fit.rejected

        if not rejected:
            if fit.inlier_ratio < config.min_inlier_ratio:
                rejected = True
                rejection_reason = (
                    f"内点率不足: {fit.inlier_ratio:.2f} < {config.min_inlier_ratio}"
                )
            elif fit.deceleration <= 0.0:
                rejected = True
                rejection_reason = "減速度が非正"

        ci_v0 = (fit.v0, fit.v0)
        ci_decel = (fit.deceleration, fit.deceleration)
        if not rejected and config.bootstrap_n > 0 and len(t_arr) >= 5:
            v0_samples, decel_samples = bootstrap_ci(
                t_arr, v_arr, fit_fn, n_boot=config.bootstrap_n
            )
            if v0_samples:
                ci_v0 = (
                    float(np.percentile(v0_samples, 2.5)),
                    float(np.percentile(v0_samples, 97.5)),
                )
                ci_decel = (
                    float(np.percentile(decel_samples, 2.5)),
                    float(np.percentile(decel_samples, 97.5)),
                )

        per_traj.append(
            PerTrajectoryFit(
                event_id=traj.event_id,
                method=fit.method,
                v0=fit.v0,
                deceleration=fit.deceleration,
                r_squared=fit.r_squared,
                rmse=fit.rmse,
                inlier_ratio=fit.inlier_ratio,
                weights=fit.weights,
                residuals=fit.residuals,
                ci_v0=ci_v0,
                ci_decel=ci_decel,
                rejected=rejected,
                rejection_reason=rejection_reason,
            )
        )
        fits.append(fit)
        n_points_list.append(len(traj.time_points))

    result.per_trajectory_fits = per_traj

    agg_stats = aggregate_deceleration(
        fits,
        n_points_list,
        method=config.aggregation_method,
        bootstrap_n=config.bootstrap_n,
    )
    result.aggregate_stats = agg_stats
    result.global_deceleration = agg_stats.deceleration

    valid_fits = [f for f in fits if not f.rejected]
    if valid_fits:
        result.global_rmse = float(np.mean([f.rmse for f in valid_fits]))
        result.global_r_squared = float(np.mean([f.r_squared for f in valid_fits]))
    result.success = agg_stats.n_trajectories >= 1 and agg_stats.deceleration > 0.0

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

        expected = [-deceleration * t for t in traj.time_points]
        _, v0, _ = _perform_linear_regression(expected, traj.velocities)

        predicted = [max(0.0, v0 - deceleration * t) for t in traj.time_points]

        mean_v = sum(traj.velocities) / len(traj.velocities)
        ss_tot = sum((v - mean_v) ** 2 for v in traj.velocities)
        ss_res = sum((a - p) ** 2 for a, p in zip(traj.velocities, predicted))
        r_sq = 1.0 - ss_res / ss_tot if ss_tot > 1e-10 else 0.0

        results.append(
            PredictedTrajectory(
                event_id=traj.event_id,
                time_points=traj.time_points,
                actual_velocities=traj.velocities,
                predicted_velocities=predicted,
                estimated_v0=v0,
                r_squared=r_sq,
            )
        )
    return results
