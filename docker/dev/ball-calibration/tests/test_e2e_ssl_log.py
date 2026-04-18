"""SSL 公式ログを使った E2E テスト.

環境変数 CRANE_TEST_LOG にパスを指定したときのみ実行。
例: CRANE_TEST_LOG=/home/hans/ダウンロード/2026-03-22_06-27_UNKNOWN_MATCH_TRAPS-vs-ibis.log.gz
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

LOG_PATH = os.environ.get("CRANE_TEST_LOG")
skip_if_no_log = pytest.mark.skipif(not LOG_PATH, reason="CRANE_TEST_LOG 未設定")


@skip_if_no_log
class TestE2ESSLLog:
    @pytest.fixture(scope="class")
    def extracted(self):
        from ssl_log_extractor import extract_ball_timeline_and_trajectories

        path = Path(LOG_PATH)
        assert path.exists(), f"ログが見つかりません: {path}"
        return extract_ball_timeline_and_trajectories(path)

    def test_trajectories_extracted(self, extracted):
        trajectories, ball_data = extracted
        assert len(trajectories) > 0, "軌道が1件も抽出されなかった"
        assert len(ball_data) > 0, "ボールデータが抽出されなかった"

    def test_ball_data_format(self, extracted):
        _, ball_data = extracted
        for entry in ball_data[:10]:
            assert len(entry) == 9, "ball_data タプルは9要素必要"
            ts_ns, x, y, z, vx, vy, vz, state, detected = entry
            assert isinstance(ts_ns, int), "timestamp は int"
            assert isinstance(state, int) and 0 <= state <= 2, "state は 0/1/2"
            assert isinstance(detected, bool), "detected は bool"

    def test_trajectory_format(self, extracted):
        trajectories, _ = extracted
        for traj in trajectories[:5]:
            assert traj.event_id >= 0
            assert len(traj.time_points) >= 10
            assert len(traj.positions_x) == len(traj.time_points)
            assert len(traj.positions_y) == len(traj.time_points)
            assert len(traj.velocities) == len(traj.time_points)
            assert traj.max_velocity > 0.0

    def test_linear_optimization(self, extracted):
        from models import OptimizationConfig
        from optimizer import run_optimization

        trajectories, _ = extracted
        cfg = OptimizationConfig(algorithm="linear")
        result = run_optimization(trajectories, cfg)
        assert result.global_deceleration >= 0.0, "linear deceleration が負値"

    def test_huber_optimization(self, extracted):
        from models import OptimizationConfig
        from optimizer import run_optimization

        trajectories, _ = extracted
        cfg = OptimizationConfig(
            algorithm="huber", aggregation_method="weighted_median"
        )
        result = run_optimization(trajectories, cfg)
        if len(trajectories) >= 5:
            assert result.global_deceleration > 0.0, (
                "huber deceleration が推定されなかった"
            )

    def test_deceleration_physical_range(self, extracted):
        """推定減速度が物理的妥当範囲 (0.05–5.0 m/s²) 内であること."""
        from models import OptimizationConfig
        from optimizer import run_optimization

        trajectories, _ = extracted
        for algo in ["linear", "huber"]:
            cfg = OptimizationConfig(algorithm=algo)
            result = run_optimization(trajectories, cfg)
            if result.global_deceleration > 0.0:
                assert 0.05 <= result.global_deceleration <= 5.0, (
                    f"{algo}: decel={result.global_deceleration} が物理的にあり得ない範囲"
                )
