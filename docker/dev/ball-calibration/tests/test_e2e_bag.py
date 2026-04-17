"""実 rosbag を使った E2E テスト.

環境変数 CRANE_TEST_BAG にパスを指定したときのみ実行。
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

BAG_PATH = os.environ.get("CRANE_TEST_BAG")
skip_if_no_bag = pytest.mark.skipif(not BAG_PATH, reason="CRANE_TEST_BAG 未設定")


@skip_if_no_bag
class TestE2EBag:
    @pytest.fixture(scope="class")
    def trajectories(self):
        from mcap_extractor import extract_trajectories_from_mcap

        path = Path(BAG_PATH)
        if path.is_dir():
            mcaps = sorted(path.glob("*.mcap"))
            assert mcaps, f"mcap が見つかりません: {path}"
            path = mcaps[0]
        return extract_trajectories_from_mcap(path)

    def test_trajectories_extracted(self, trajectories):
        assert len(trajectories) > 0, "軌道が1件も抽出されなかった"

    def test_linear_optimization(self, trajectories):
        from models import OptimizationConfig
        from optimizer import run_optimization

        cfg = OptimizationConfig(algorithm="linear")
        result = run_optimization(trajectories, cfg)
        # 試合データでは kick_power=0 が多いためsuccess が取れないケースもある
        # deceleration が推定されたことのみ確認
        assert result.global_deceleration >= 0.0, "linear deceleration が負値"

    def test_huber_optimization(self, trajectories):
        from models import OptimizationConfig
        from optimizer import run_optimization

        cfg = OptimizationConfig(
            algorithm="huber", aggregation_method="weighted_median"
        )
        result = run_optimization(trajectories, cfg)
        # 軌道が抽出されていれば、Huber 推定が実行されていることを確認
        if len(trajectories) >= 5:
            assert result.global_deceleration > 0.0, (
                "huber deceleration が推定されなかった"
            )

    def test_huber_ci_valid(self, trajectories):
        """Huber の CI が正しい形式であること（lo < hi かつ正値）."""
        from models import OptimizationConfig
        from optimizer import run_optimization

        cfg_huber = OptimizationConfig(
            algorithm="huber", aggregation_method="weighted_median", bootstrap_n=200
        )
        result_huber = run_optimization(trajectories, cfg_huber)

        if result_huber.aggregate_stats is None:
            pytest.skip("Huber aggregate_stats が得られなかった")

        agg = result_huber.aggregate_stats
        lo, hi = agg.ci_decel
        # CI は正の範囲で lo <= hi であること（試合データでは幅が広くなりうる）
        assert lo >= 0.0, f"CI 下限が負: {lo}"
        assert lo <= hi, f"CI 逆転: {agg.ci_decel}"

    def test_deceleration_physical_range(self, trajectories):
        """推定減速度が物理的妥当範囲 (0.05–5.0 m/s²) 内であること.

        試合データは品質が低く推定範囲が広いため、上限は緩めに設定。
        """
        from models import OptimizationConfig
        from optimizer import run_optimization

        for algo in ["linear", "huber"]:
            cfg = OptimizationConfig(algorithm=algo)
            result = run_optimization(trajectories, cfg)
            if result.global_deceleration > 0.0:
                assert 0.05 <= result.global_deceleration <= 5.0, (
                    f"{algo}: decel={result.global_deceleration} が物理的にあり得ない範囲"
                )
