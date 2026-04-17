"""軌道単体のロバストフィットコア.

Huber / RANSAC / 非線形 Huber / 指数減衰モデルによる
v(t) の勾配・切片推定とブートストラップ CI。
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np
from scipy.optimize import least_squares

logger = logging.getLogger(__name__)


@dataclass
class FitResult:
    """軌道フィット結果."""

    slope: float = 0.0
    intercept: float = 0.0
    r_squared: float = 0.0
    rmse: float = float("inf")
    inlier_ratio: float = 1.0
    weights: list[float] = field(default_factory=list)
    residuals: list[float] = field(default_factory=list)
    v0: float = 0.0
    deceleration: float = 0.0
    ci_v0: tuple[float, float] = (0.0, 0.0)
    ci_decel: tuple[float, float] = (0.0, 0.0)
    method: str = "unknown"
    rejected: bool = False
    rejection_reason: str | None = None


def _r_squared(actual: np.ndarray, predicted: np.ndarray) -> float:
    ss_res = float(np.sum((actual - predicted) ** 2))
    ss_tot = float(np.sum((actual - np.mean(actual)) ** 2))
    return 1.0 - ss_res / ss_tot if ss_tot > 1e-10 else 0.0


def _rmse(actual: np.ndarray, predicted: np.ndarray) -> float:
    return float(np.sqrt(np.mean((actual - predicted) ** 2)))


def fit_linear_huber(
    time_points: np.ndarray,
    velocities: np.ndarray,
    epsilon: float = 1.35,
) -> FitResult:
    """HuberRegressor による線形フィット v(t) = v0 - a*t."""
    from sklearn.linear_model import HuberRegressor

    result = FitResult(method="huber")
    n = len(time_points)
    if n < 3:
        result.rejected = True
        result.rejection_reason = f"点数不足: {n} < 3"
        return result

    X = time_points.reshape(-1, 1)
    try:
        model = HuberRegressor(epsilon=epsilon, max_iter=300)
        model.fit(X, velocities)
    except Exception as e:
        result.method = "huber_failed"
        result.rejected = True
        result.rejection_reason = f"HuberRegressor 収束失敗: {e}"
        return result

    predicted = model.predict(X)
    residuals = velocities - predicted
    inlier_ratio = (
        float(np.sum(~model.outliers_) / n) if hasattr(model, "outliers_") else 1.0
    )

    result.slope = float(model.coef_[0])
    result.intercept = float(model.intercept_)
    result.v0 = float(model.intercept_)
    result.deceleration = -float(model.coef_[0])
    result.r_squared = _r_squared(velocities, predicted)
    result.rmse = _rmse(velocities, predicted)
    result.residuals = residuals.tolist()
    result.inlier_ratio = inlier_ratio
    result.weights = (
        (~model.outliers_).astype(float).tolist()
        if hasattr(model, "outliers_")
        else [1.0] * n
    )
    return result


def fit_linear_ransac(
    time_points: np.ndarray,
    velocities: np.ndarray,
    residual_threshold: float | None = None,
    min_samples: int = 5,
) -> FitResult:
    """RANSACRegressor による線形フィット v(t) = v0 - a*t."""
    from sklearn.linear_model import RANSACRegressor
    from sklearn.linear_model import LinearRegression

    result = FitResult(method="ransac")
    n = len(time_points)
    if n < max(min_samples + 1, 5):
        result.rejected = True
        result.rejection_reason = f"点数不足: {n} < {max(min_samples + 1, 5)}"
        return result

    X = time_points.reshape(-1, 1)
    threshold = residual_threshold or (np.std(velocities) * 1.5)

    try:
        model = RANSACRegressor(
            estimator=LinearRegression(),
            min_samples=min_samples,
            residual_threshold=threshold,
            random_state=0,
        )
        model.fit(X, velocities)
    except Exception as e:
        result.method = "ransac_failed"
        result.rejected = True
        result.rejection_reason = f"RANSAC 失敗: {e}"
        return result

    inlier_mask: np.ndarray = model.inlier_mask_
    predicted = model.predict(X)
    residuals = velocities - predicted

    result.slope = float(model.estimator_.coef_[0])
    result.intercept = float(model.estimator_.intercept_)
    result.v0 = float(model.estimator_.intercept_)
    result.deceleration = -float(model.estimator_.coef_[0])
    result.r_squared = _r_squared(velocities, predicted)
    result.rmse = (
        _rmse(velocities[inlier_mask], predicted[inlier_mask])
        if inlier_mask.any()
        else float("inf")
    )
    result.residuals = residuals.tolist()
    result.inlier_ratio = float(np.sum(inlier_mask) / n)
    result.weights = inlier_mask.astype(float).tolist()
    return result


def fit_nonlinear_huber(
    time_points: np.ndarray,
    velocities: np.ndarray,
    loss: str = "soft_l1",
) -> FitResult:
    """scipy least_squares (soft_l1/huber) による非線形フィット v(t) = v0 - a*t."""
    result = FitResult(method=f"nonlinear_{loss}")
    n = len(time_points)
    if n < 3:
        result.rejected = True
        result.rejection_reason = f"点数不足: {n} < 3"
        return result

    def residuals_fn(params: np.ndarray) -> np.ndarray:
        v0, a = params
        return velocities - (v0 - a * time_points)

    # 初期値: 線形回帰の結果から
    from optimizer import _perform_linear_regression

    slope_init, intercept_init, _ = _perform_linear_regression(
        time_points.tolist(), velocities.tolist()
    )
    x0 = [max(0.1, intercept_init), max(0.01, -slope_init)]

    try:
        res = least_squares(
            residuals_fn,
            x0=x0,
            loss=loss,
            bounds=([0.0, 0.0], [np.inf, np.inf]),
        )
    except Exception as e:
        result.method = f"nonlinear_{loss}_failed"
        result.rejected = True
        result.rejection_reason = f"least_squares 失敗: {e}"
        return result

    v0_fit, a_fit = float(res.x[0]), float(res.x[1])
    predicted = v0_fit - a_fit * time_points
    residuals = velocities - predicted

    # コスト関数から内点率を推定（残差が中央値絶対偏差の3倍以内）
    mad = float(np.median(np.abs(residuals - np.median(residuals))))
    inlier_mask = np.abs(residuals) < (3.0 * mad + 1e-6)
    inlier_ratio = float(np.sum(inlier_mask) / n)

    result.slope = -a_fit
    result.intercept = v0_fit
    result.v0 = v0_fit
    result.deceleration = a_fit
    result.r_squared = _r_squared(velocities, predicted)
    result.rmse = _rmse(velocities, predicted)
    result.residuals = residuals.tolist()
    result.inlier_ratio = inlier_ratio
    result.weights = inlier_mask.astype(float).tolist()
    return result


def fit_exponential_decay(
    time_points: np.ndarray,
    velocities: np.ndarray,
) -> FitResult:
    """指数減衰フィット v(t) = v0 * exp(-k*t)."""
    result = FitResult(method="exponential_decay")
    n = len(time_points)
    if n < 3:
        result.rejected = True
        result.rejection_reason = f"点数不足: {n} < 3"
        return result

    # log 変換で初期値を推定（正値のみ）
    pos_mask = velocities > 0.0
    if np.sum(pos_mask) < 3:
        result.rejected = True
        result.rejection_reason = "正速度点数不足（指数フィット不可）"
        return result

    def residuals_fn(params: np.ndarray) -> np.ndarray:
        v0, k = params
        return velocities - v0 * np.exp(-k * time_points)

    log_v = np.log(np.where(velocities > 0, velocities, 1e-6))
    from optimizer import _perform_linear_regression

    slope_log, intercept_log, _ = _perform_linear_regression(
        time_points[pos_mask].tolist(), log_v[pos_mask].tolist()
    )
    x0 = [max(0.1, math.exp(intercept_log)), max(0.01, -slope_log)]

    try:
        res = least_squares(
            residuals_fn,
            x0=x0,
            loss="soft_l1",
            bounds=([0.0, 0.0], [np.inf, np.inf]),
        )
    except Exception as e:
        result.method = "exponential_decay_failed"
        result.rejected = True
        result.rejection_reason = f"least_squares 失敗: {e}"
        return result

    v0_fit, k_fit = float(res.x[0]), float(res.x[1])
    predicted = v0_fit * np.exp(-k_fit * time_points)
    residuals = velocities - predicted

    mad = float(np.median(np.abs(residuals - np.median(residuals))))
    inlier_mask = np.abs(residuals) < (3.0 * mad + 1e-6)

    result.slope = -k_fit * v0_fit  # 初速付近の線形近似勾配
    result.intercept = v0_fit
    result.v0 = v0_fit
    result.deceleration = k_fit  # 指数モデルでは k が減衰係数 [1/s]
    result.r_squared = _r_squared(velocities, predicted)
    result.rmse = _rmse(velocities, predicted)
    result.residuals = residuals.tolist()
    result.inlier_ratio = float(np.sum(inlier_mask) / n)
    result.weights = inlier_mask.astype(float).tolist()
    return result


def bootstrap_ci(
    time_points: np.ndarray,
    velocities: np.ndarray,
    fit_fn: Callable[[np.ndarray, np.ndarray], FitResult],
    n_boot: int = 300,
    rng_seed: int = 0,
) -> tuple[list[float], list[float]]:
    """ブートストラップで v0 と deceleration の分布を返す.

    Returns:
        (v0_samples, decel_samples): 各ブートストラップ推定値のリスト
    """
    rng = np.random.default_rng(rng_seed)
    n = len(time_points)
    v0_samples, decel_samples = [], []

    for _ in range(n_boot):
        idx = rng.integers(0, n, size=n)
        t_boot = time_points[idx]
        v_boot = velocities[idx]
        r = fit_fn(t_boot, v_boot)
        if not r.rejected and r.v0 > 0.0 and r.deceleration > 0.0:
            v0_samples.append(r.v0)
            decel_samples.append(r.deceleration)

    return v0_samples, decel_samples


def pick_fit_fn(algorithm: str, physics_model: str, **kwargs) -> Callable:
    """アルゴリズムと物理モデルから対応するフィット関数を選ぶ."""
    if physics_model == "exponential_decay":
        return fit_exponential_decay

    if algorithm == "huber":
        epsilon = kwargs.get("epsilon", 1.35)
        return lambda t, v: fit_linear_huber(t, v, epsilon=epsilon)
    if algorithm == "ransac":
        threshold = kwargs.get("residual_threshold", None)
        return lambda t, v: fit_linear_ransac(t, v, residual_threshold=threshold)
    if algorithm == "nonlinear_huber":
        return lambda t, v: fit_nonlinear_huber(t, v, loss="soft_l1")

    # fallback: 線形（optimizer._perform_linear_regression ラッパ）
    return _fit_linear_ols


def _fit_linear_ols(
    time_points: np.ndarray,
    velocities: np.ndarray,
) -> FitResult:
    """標準線形回帰（OLS）のラッパ."""
    from optimizer import _perform_linear_regression

    result = FitResult(method="linear")
    n = len(time_points)
    slope, intercept, r_sq = _perform_linear_regression(
        time_points.tolist(), velocities.tolist()
    )
    predicted = intercept + slope * time_points
    residuals = velocities - predicted

    result.slope = slope
    result.intercept = intercept
    result.v0 = intercept
    result.deceleration = -slope
    result.r_squared = r_sq
    result.rmse = _rmse(velocities, predicted)
    result.residuals = residuals.tolist()
    result.weights = [1.0] * n
    result.inlier_ratio = 1.0
    return result
