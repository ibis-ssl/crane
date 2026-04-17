"""aggregate.py のユニットテスト."""

from __future__ import annotations

from aggregate import aggregate_deceleration, trajectory_weight, weighted_median_decel
from robust_fit import FitResult


def _make_fit(
    decel: float, r2: float = 0.9, inlier: float = 0.9, rejected: bool = False
) -> FitResult:
    return FitResult(
        deceleration=decel,
        r_squared=r2,
        inlier_ratio=inlier,
        rejected=rejected,
        v0=4.0,
        rmse=0.1,
        method="huber",
    )


class TestTrajectoryWeight:
    def test_high_quality_high_weight(self):
        fit = _make_fit(0.7, r2=0.95, inlier=0.95)
        w = trajectory_weight(fit, 50)
        assert w > trajectory_weight(_make_fit(0.7, r2=0.5, inlier=0.5), 50)

    def test_weight_capped_at_100(self):
        fit = _make_fit(0.7, r2=1.0, inlier=1.0)
        w = trajectory_weight(fit, 1000)
        assert w <= 100.0


class TestWeightedMedian:
    def test_single_value(self):
        fits = [_make_fit(0.7)]
        ws = [1.0]
        result = weighted_median_decel(fits, ws)
        assert abs(result - 0.7) < 1e-6

    def test_outlier_suppressed(self):
        """外れ値が高重みなら影響が出るが、低重みなら中央値は正しい."""
        fits = [_make_fit(0.7), _make_fit(0.7), _make_fit(0.7), _make_fit(5.0)]
        ws = [1.0, 1.0, 1.0, 0.01]  # 外れ値は低重み
        result = weighted_median_decel(fits, ws)
        assert abs(result - 0.7) < 0.1


class TestAggregateDeceleration:
    def test_basic_weighted_median(self):
        fits = [_make_fit(0.7 + 0.02 * i) for i in range(5)]
        n_pts = [40] * 5
        stats = aggregate_deceleration(fits, n_pts, method="weighted_median")
        assert stats.deceleration > 0.0
        assert stats.n_trajectories == 5
        assert 0.0 <= stats.inlier_trajectory_ratio <= 1.0

    def test_rejected_fits_excluded(self):
        fits = [_make_fit(0.7), _make_fit(0.7), _make_fit(5.0, rejected=True)]
        n_pts = [40, 40, 40]
        stats = aggregate_deceleration(fits, n_pts, method="weighted_median")
        assert stats.n_trajectories == 2
        assert abs(stats.deceleration - 0.7) < 0.1

    def test_ci_valid(self):
        fits = [_make_fit(0.7 + 0.01 * i) for i in range(10)]
        n_pts = [40] * 10
        stats = aggregate_deceleration(fits, n_pts, bootstrap_n=200)
        lo, hi = stats.ci_decel
        assert lo <= stats.deceleration <= hi

    def test_all_rejected_returns_zero(self):
        fits = [_make_fit(0.7, rejected=True)] * 3
        n_pts = [40] * 3
        stats = aggregate_deceleration(fits, n_pts)
        assert stats.deceleration == 0.0
        assert stats.n_trajectories == 0

    def test_mm_method(self):
        fits = [_make_fit(0.7 + 0.01 * i) for i in range(8)]
        n_pts = [40] * 8
        stats = aggregate_deceleration(fits, n_pts, method="mm")
        assert abs(stats.deceleration - 0.735) < 0.1
