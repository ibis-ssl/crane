"""optimizer.py の単体テスト."""

from crane_ball_calibration_ui.models import OptimizationConfig, TrajectoryData
from crane_ball_calibration_ui.optimizer import (
    _perform_linear_regression,
    compute_predicted_trajectories,
    filter_quality_data,
    optimize_global_deceleration,
    run_optimization,
)


def make_synthetic_trajectory(
    v0: float,
    decel: float,
    kick_power: float = 0.5,
    duration: float = 1.5,
    dt: float = 0.05,
    event_id: int = 1,
) -> TrajectoryData:
    """合成軌道データを生成する (v(t) = v0 - decel*t)."""
    time_points = []
    velocities = []
    positions_x = []
    positions_y = []
    t = 0.0
    x = 0.0
    while t <= duration:
        v = max(0.0, v0 - decel * t)
        time_points.append(t)
        velocities.append(v)
        positions_x.append(x)
        positions_y.append(0.0)
        x += v * dt
        t = round(t + dt, 6)

    return TrajectoryData(
        event_id=event_id,
        kick_power=kick_power,
        is_chip_kick=False,
        time_points=time_points,
        velocities=velocities,
        positions_x=positions_x,
        positions_y=positions_y,
    )


class TestLinearRegression:
    def test_perfect_fit(self):
        x = [0.0, 1.0, 2.0, 3.0]
        y = [2.0, 4.0, 6.0, 8.0]
        slope, intercept, r2 = _perform_linear_regression(x, y)
        assert abs(slope - 2.0) < 1e-9
        assert abs(intercept - 2.0) < 1e-9
        assert abs(r2 - 1.0) < 1e-9

    def test_deceleration_line(self):
        # v(t) = 5.0 - 0.7 * t
        v0, decel = 5.0, 0.7
        t = [i * 0.1 for i in range(20)]
        v = [v0 - decel * ti for ti in t]
        slope, intercept, r2 = _perform_linear_regression(t, v)
        assert abs(slope - (-decel)) < 1e-6
        assert abs(intercept - v0) < 1e-6
        assert abs(r2 - 1.0) < 1e-6

    def test_too_few_points(self):
        slope, intercept, r2 = _perform_linear_regression([1.0], [1.0])
        assert slope == 0.0
        assert intercept == 0.0
        assert r2 == 0.0


class TestFilterQualityData:
    def setup_method(self):
        self.config = OptimizationConfig()

    def test_straight_kick_passes(self):
        traj = make_synthetic_trajectory(5.0, 0.7)
        result = filter_quality_data([traj], self.config)
        assert len(result) == 1

    def test_chip_kick_excluded(self):
        traj = make_synthetic_trajectory(5.0, 0.7)
        traj.is_chip_kick = True
        result = filter_quality_data([traj], self.config)
        assert len(result) == 0

    def test_short_trajectory_excluded(self):
        traj = make_synthetic_trajectory(5.0, 0.7, duration=0.2)
        result = filter_quality_data([traj], self.config)
        assert len(result) == 0

    def test_invalid_kick_power_excluded(self):
        traj = make_synthetic_trajectory(5.0, 0.7)
        traj.kick_power = 0.0
        result = filter_quality_data([traj], self.config)
        assert len(result) == 0

    def test_zero_velocity_excluded(self):
        traj = make_synthetic_trajectory(0.3, 0.7)  # max_velocity < 0.5
        result = filter_quality_data([traj], self.config)
        assert len(result) == 0


class TestOptimizeGlobalDeceleration:
    def test_recovers_true_deceleration(self):
        """合成データから真の減速度を回復できるか."""
        true_decel = 0.73
        trajectories = [
            make_synthetic_trajectory(3.0, true_decel, kick_power=0.3, event_id=1),
            make_synthetic_trajectory(5.0, true_decel, kick_power=0.5, event_id=2),
            make_synthetic_trajectory(7.0, true_decel, kick_power=0.8, event_id=3),
        ]
        config = OptimizationConfig()
        decel, rmse = optimize_global_deceleration(trajectories, config)
        # 0.01刻みのグリッドサーチなので ±0.01 以内に収まるはず
        assert abs(decel - true_decel) <= 0.01
        assert rmse < 0.1


class TestRunOptimization:
    def test_full_pipeline(self):
        """最適化パイプライン全体の動作確認."""
        true_decel = 0.70
        trajectories = [
            make_synthetic_trajectory(4.0, true_decel, kick_power=0.4, event_id=i)
            for i in range(5)
        ]
        config = OptimizationConfig(min_fitting_r_squared=0.5)
        result = run_optimization(trajectories, config)

        assert result.success
        assert abs(result.global_deceleration - true_decel) <= 0.02
        assert result.trajectories_used == 5
        assert len(result.kick_data) > 0

    def test_event_id_selection(self):
        """選択した event_id のみが使用されるか."""
        true_decel = 0.70
        trajectories = [
            make_synthetic_trajectory(5.0, true_decel, kick_power=0.5, event_id=i)
            for i in range(6)
        ]
        config = OptimizationConfig(min_fitting_r_squared=0.5)
        result = run_optimization(trajectories, config, enabled_event_ids=[0, 1, 2, 3])
        assert result.trajectories_used <= 4

    def test_insufficient_data(self):
        """データ不足の場合はsuccess=Falseを返す."""
        traj = make_synthetic_trajectory(5.0, 0.7, event_id=1)
        config = OptimizationConfig()
        result = run_optimization([traj], config)
        assert not result.success


class TestComputePredictedTrajectories:
    def test_prediction_accuracy(self):
        """予測軌道の精度確認."""
        true_decel = 0.73
        v0 = 5.0
        traj = make_synthetic_trajectory(v0, true_decel, event_id=1)
        config = OptimizationConfig()
        predicted = compute_predicted_trajectories([traj], true_decel, config)

        assert len(predicted) == 1
        p = predicted[0]
        assert p.r_squared > 0.99
        assert abs(p.estimated_v0 - v0) < 0.1
