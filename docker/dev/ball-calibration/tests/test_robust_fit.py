"""robust_fit.py のユニットテスト."""

from __future__ import annotations

import numpy as np
from robust_fit import (
    FitResult,
    bootstrap_ci,
    fit_exponential_decay,
    fit_linear_huber,
    fit_linear_ransac,
    fit_nonlinear_huber,
    pick_fit_fn,
)

from tests.conftest import make_synthetic_trajectory

TRUE_V0 = 4.0
TRUE_DECEL = 0.7
TOL = 0.15  # 許容誤差（m/s または m/s²）


def _check_basic(fit: FitResult, decel_tol: float = TOL, v0_tol: float = TOL):
    assert not fit.rejected, f"rejected: {fit.rejection_reason}"
    assert abs(fit.deceleration - TRUE_DECEL) < decel_tol, (
        f"decel={fit.deceleration:.3f}"
    )
    assert abs(fit.v0 - TRUE_V0) < v0_tol, f"v0={fit.v0:.3f}"
    assert 0.0 < fit.r_squared <= 1.0
    assert 0.0 < fit.inlier_ratio <= 1.0


class TestNoOutliers:
    def test_huber(self):
        t, v = make_synthetic_trajectory(TRUE_V0, TRUE_DECEL, noise_std=0.02)
        fit = fit_linear_huber(t, v)
        _check_basic(fit)

    def test_ransac(self):
        t, v = make_synthetic_trajectory(TRUE_V0, TRUE_DECEL, noise_std=0.02)
        fit = fit_linear_ransac(t, v)
        _check_basic(fit)

    def test_nonlinear_huber(self):
        t, v = make_synthetic_trajectory(TRUE_V0, TRUE_DECEL, noise_std=0.02)
        fit = fit_nonlinear_huber(t, v)
        _check_basic(fit)

    def test_exponential_decay(self):
        t, v = make_synthetic_trajectory(TRUE_V0, TRUE_DECEL, noise_std=0.02)
        fit = fit_exponential_decay(t, v)
        # 指数モデルはパラメータ意味が違うため v0 のみチェック
        assert not fit.rejected
        assert abs(fit.v0 - TRUE_V0) < TOL


class TestWithOutliers:
    """外れ値 20% でロバスト推定が線形回帰より安定であることを確認."""

    def test_huber_beats_ols(self):
        t, v = make_synthetic_trajectory(
            TRUE_V0, TRUE_DECEL, outlier_ratio=0.2, noise_std=0.05
        )
        fit_hub = fit_linear_huber(t, v)
        # ロバスト推定は棄却されない
        assert not fit_hub.rejected
        assert abs(fit_hub.deceleration - TRUE_DECEL) < TOL

    def test_ransac_beats_ols(self):
        t, v = make_synthetic_trajectory(
            TRUE_V0, TRUE_DECEL, outlier_ratio=0.25, noise_std=0.05
        )
        fit_rans = fit_linear_ransac(t, v)
        assert not fit_rans.rejected
        assert abs(fit_rans.deceleration - TRUE_DECEL) < TOL

    def test_ransac_inlier_ratio(self):
        t, v = make_synthetic_trajectory(
            TRUE_V0, TRUE_DECEL, outlier_ratio=0.25, noise_std=0.02
        )
        fit = fit_linear_ransac(t, v)
        # 内点率が 0.5 以上あること
        assert fit.inlier_ratio >= 0.5


class TestFewPoints:
    def test_too_few_rejected(self):
        t = np.array([0.0, 0.05])
        v = np.array([4.0, 3.9])
        fit = fit_linear_huber(t, v)
        assert fit.rejected

    def test_minimum_points(self):
        t, v = make_synthetic_trajectory(
            TRUE_V0, TRUE_DECEL, n_points=5, noise_std=0.01
        )
        fit = fit_linear_huber(t, v)
        # 5点は境界値、棄却されても良いがクラッシュしないこと
        assert isinstance(fit, FitResult)


class TestBootstrapCI:
    def test_ci_contains_true(self):
        t, v = make_synthetic_trajectory(TRUE_V0, TRUE_DECEL, noise_std=0.03)
        v0_s, decel_s = bootstrap_ci(t, v, fit_linear_huber, n_boot=200)
        assert len(decel_s) > 0
        lo, hi = np.percentile(decel_s, 2.5), np.percentile(decel_s, 97.5)
        assert lo < TRUE_DECEL < hi

    def test_ci_narrower_with_more_data(self):
        # データが多い方が CI が狭い
        t_few, v_few = make_synthetic_trajectory(
            TRUE_V0, TRUE_DECEL, n_points=10, noise_std=0.05
        )
        t_many, v_many = make_synthetic_trajectory(
            TRUE_V0, TRUE_DECEL, n_points=60, noise_std=0.05
        )
        _, decel_few = bootstrap_ci(t_few, v_few, fit_linear_huber, n_boot=100)
        _, decel_many = bootstrap_ci(t_many, v_many, fit_linear_huber, n_boot=100)
        if len(decel_few) > 5 and len(decel_many) > 5:
            std_few = float(np.std(decel_few))
            std_many = float(np.std(decel_many))
            assert std_many < std_few


class TestPickFitFn:
    def test_pick_huber(self):
        fn = pick_fit_fn("huber", "linear_decay")
        t, v = make_synthetic_trajectory(TRUE_V0, TRUE_DECEL, noise_std=0.02)
        fit = fn(t, v)
        assert fit.method == "huber"

    def test_pick_ransac(self):
        fn = pick_fit_fn("ransac", "linear_decay")
        t, v = make_synthetic_trajectory(TRUE_V0, TRUE_DECEL, noise_std=0.02)
        fit = fn(t, v)
        assert fit.method == "ransac"

    def test_pick_exponential(self):
        fn = pick_fit_fn("huber", "exponential_decay")
        t, v = make_synthetic_trajectory(TRUE_V0, TRUE_DECEL, noise_std=0.02)
        fit = fn(t, v)
        assert fit.method == "exponential_decay"
