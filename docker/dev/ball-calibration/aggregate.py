"""軌道横断の集約モジュール.

複数軌道から推定された deceleration を、重み付き中央値または MM 推定で集約する。
"""

from __future__ import annotations

import logging
from typing import Literal

import numpy as np

from models import RobustAggregateStats
from robust_fit import FitResult

logger = logging.getLogger(__name__)


def trajectory_weight(fit: FitResult, n_points: int) -> float:
    """軌道の重みスコアを計算: n * R² * inlier_ratio (上限クリップ)."""
    r2 = max(0.0, fit.r_squared)
    score = n_points * r2 * fit.inlier_ratio
    return min(score, 100.0)


def weighted_median_decel(
    fits: list[FitResult],
    weights: list[float],
) -> float:
    """重み付き中央値で deceleration を集約."""
    pairs = sorted(zip([f.deceleration for f in fits], weights), key=lambda x: x[0])
    decels = [p[0] for p in pairs]
    ws = [p[1] for p in pairs]
    total = sum(ws)
    if total < 1e-10:
        return float(np.median([f.deceleration for f in fits]))

    cumsum = 0.0
    for d, w in zip(decels, ws):
        cumsum += w
        if cumsum >= total / 2.0:
            return d
    return decels[-1]


def _bootstrap_ci_from_samples(
    values: list[float],
    n_boot: int = 500,
    alpha: float = 0.05,
    rng_seed: int = 0,
) -> tuple[float, float]:
    """ブートストラップによる信頼区間."""
    if len(values) < 3:
        med = float(np.median(values)) if values else 0.0
        return med, med
    arr = np.array(values)
    rng = np.random.default_rng(rng_seed)
    boot_medians = [
        np.median(rng.choice(arr, size=len(arr), replace=True)) for _ in range(n_boot)
    ]
    lo = float(np.percentile(boot_medians, 100 * alpha / 2))
    hi = float(np.percentile(boot_medians, 100 * (1 - alpha / 2)))
    return lo, hi


def aggregate_deceleration(
    fits: list[FitResult],
    n_points_list: list[int],
    method: Literal["weighted_median", "mm", "grid"] = "weighted_median",
    bootstrap_n: int = 300,
) -> RobustAggregateStats:
    """複数軌道の FitResult から deceleration を集約する."""
    valid = [
        (f, n)
        for f, n in zip(fits, n_points_list)
        if not f.rejected and f.deceleration > 0.0 and f.r_squared > 0.0
    ]
    if not valid:
        return RobustAggregateStats(
            method=method,
            deceleration=0.0,
            ci_decel=(0.0, 0.0),
            n_trajectories=0,
            inlier_trajectory_ratio=0.0,
        )

    valid_fits = [v[0] for v in valid]
    valid_ns = [v[1] for v in valid]
    ws = [trajectory_weight(f, n) for f, n in zip(valid_fits, valid_ns)]

    decels = [f.deceleration for f in valid_fits]

    if method == "weighted_median":
        decel_est = weighted_median_decel(valid_fits, ws)
    elif method == "mm":
        # MM 推定: 重み付き平均 + Huber 反復（簡易実装: 1ステップ Huber 重み付き平均）
        decel_est = _mm_estimate(decels, ws)
    else:
        # grid: 単純重み付き平均（旧互換）
        total_w = sum(ws)
        decel_est = (
            sum(d * w for d, w in zip(decels, ws)) / total_w
            if total_w > 0
            else float(np.mean(decels))
        )

    ci_lo, ci_hi = _bootstrap_ci_from_samples(decels, n_boot=bootstrap_n)

    inlier_ratio = len(valid) / max(len(fits), 1)

    logger.info(
        "集約完了: method=%s, decel=%.4f, CI=[%.4f, %.4f], n=%d, inlier_ratio=%.2f",
        method,
        decel_est,
        ci_lo,
        ci_hi,
        len(valid),
        inlier_ratio,
    )

    return RobustAggregateStats(
        method=method,
        deceleration=decel_est,
        ci_decel=(ci_lo, ci_hi),
        n_trajectories=len(valid),
        inlier_trajectory_ratio=inlier_ratio,
    )


def _mm_estimate(values: list[float], weights: list[float]) -> float:
    """Huber 重みを反復して求める MM 推定（簡易版）."""
    arr = np.array(values)
    w = np.array(weights, dtype=float)
    w = w / (w.sum() + 1e-10)

    est = float(np.average(arr, weights=w))
    for _ in range(10):
        residuals = np.abs(arr - est)
        sigma = max(float(np.median(residuals)) * 1.4826, 1e-6)
        c = 1.345 * sigma
        huber_w = np.where(residuals <= c, 1.0, c / (residuals + 1e-10))
        combined_w = w * huber_w
        total = combined_w.sum()
        if total < 1e-10:
            break
        est = float(np.sum(arr * combined_w) / total)

    return est
