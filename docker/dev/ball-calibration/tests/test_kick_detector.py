"""kick_detector.py のユニットテスト."""

from __future__ import annotations

from kick_detector import detect_kick_events
from models import KickDetectorConfig

_STATE_STOPPED = 0
_STATE_ROLLING = 1


def _make_ball_entry(ts_ns: int, x: float, y: float, vx: float, vy: float):
    return (ts_ns, x, y, 0.0, vx, vy, 0.0, _STATE_ROLLING, True)


def _static_seq(n: int, ts_start: int, dt_ns: int = 10_000_000):
    """静止シーケンスを生成."""
    return [
        _make_ball_entry(ts_start + i * dt_ns, 0.0, 0.0, 0.0, 0.0) for i in range(n)
    ]


class TestBasicDetection:
    def test_detects_velocity_jump(self):
        """速度ジャンプでキックを検出できる."""
        cfg = KickDetectorConfig(min_kick_speed=0.8, min_delta_v=0.5, min_accel=5.0)
        data = [
            _make_ball_entry(0, 0.0, 0.0, 0.0, 0.0),
            _make_ball_entry(10_000_000, 0.0, 0.0, 0.0, 0.0),  # 静止 (dt=10ms)
            _make_ball_entry(20_000_000, 0.0, 0.0, 3.0, 0.0),  # キック: Δv=3.0
        ]
        events = detect_kick_events(data, None, cfg)
        assert len(events) == 1
        assert events[0].v_jump > 0.5

    def test_no_detection_below_threshold(self):
        """しきい値以下では検出しない."""
        cfg = KickDetectorConfig(min_kick_speed=2.0, min_delta_v=1.5, min_accel=10.0)
        data = [
            _make_ball_entry(0, 0.0, 0.0, 0.0, 0.0),
            _make_ball_entry(10_000_000, 0.0, 0.0, 0.5, 0.0),  # 速度ジャンプ小さい
        ]
        events = detect_kick_events(data, None, cfg)
        assert len(events) == 0

    def test_cooldown_prevents_duplicates(self):
        """クールダウン内の連続ジャンプは1件だけ検出."""
        cfg = KickDetectorConfig(
            min_kick_speed=0.8, min_delta_v=0.5, min_accel=5.0, cooldown_s=0.5
        )
        data = [
            _make_ball_entry(0, 0.0, 0.0, 0.0, 0.0),
            _make_ball_entry(10_000_000, 0.0, 0.0, 3.0, 0.0),  # キック1
            _make_ball_entry(20_000_000, 0.0, 0.0, 0.0, 0.0),
            _make_ball_entry(100_000_000, 0.0, 0.0, 3.0, 0.0),  # 0.1s後: クールダウン中
        ]
        events = detect_kick_events(data, None, cfg)
        assert len(events) == 1

    def test_detects_after_cooldown(self):
        """クールダウン後は再び検出する."""
        cfg = KickDetectorConfig(
            min_kick_speed=0.8, min_delta_v=0.5, min_accel=5.0, cooldown_s=0.3
        )
        data = [
            _make_ball_entry(0, 0.0, 0.0, 0.0, 0.0),
            _make_ball_entry(10_000_000, 0.0, 0.0, 3.0, 0.0),  # キック1
            _make_ball_entry(20_000_000, 0.0, 0.0, 0.0, 0.0),
            _make_ball_entry(
                400_000_000, 0.0, 0.0, 0.0, 0.0
            ),  # 0.4s後: クールダウン終了
            _make_ball_entry(410_000_000, 0.0, 0.0, 3.0, 0.0),  # キック2
        ]
        events = detect_kick_events(data, None, cfg)
        assert len(events) == 2


class TestOwnerDetection:
    def test_owner_id_assigned_when_robot_nearby(self):
        cfg = KickDetectorConfig(
            min_kick_speed=0.8, min_delta_v=0.5, min_accel=5.0, ownership_radius=0.5
        )
        ball_data = [
            _make_ball_entry(0, 0.0, 0.0, 0.0, 0.0),
            _make_ball_entry(10_000_000, 0.0, 0.0, 3.0, 0.0),
        ]
        robot_data = [
            (10_000_000, "blue", 3, 0.1, 0.1),  # ボールに近い
        ]
        events = detect_kick_events(ball_data, robot_data, cfg)
        assert len(events) == 1
        assert events[0].owner_robot_id == 3

    def test_no_owner_when_robot_far(self):
        cfg = KickDetectorConfig(
            min_kick_speed=0.8, min_delta_v=0.5, min_accel=5.0, ownership_radius=0.1
        )
        ball_data = [
            _make_ball_entry(0, 0.0, 0.0, 0.0, 0.0),
            _make_ball_entry(10_000_000, 0.0, 0.0, 3.0, 0.0),
        ]
        robot_data = [
            (10_000_000, "blue", 3, 5.0, 5.0),  # 遠い
        ]
        events = detect_kick_events(ball_data, robot_data, cfg)
        assert len(events) == 1
        assert events[0].owner_robot_id is None

    def test_no_owner_when_robot_data_none(self):
        cfg = KickDetectorConfig(min_kick_speed=0.8, min_delta_v=0.5, min_accel=5.0)
        ball_data = [
            _make_ball_entry(0, 0.0, 0.0, 0.0, 0.0),
            _make_ball_entry(10_000_000, 0.0, 0.0, 3.0, 0.0),
        ]
        events = detect_kick_events(ball_data, None, cfg)
        assert len(events) == 1
        assert events[0].owner_robot_id is None


class TestEdgeCases:
    def test_empty_data(self):
        events = detect_kick_events([], None, None)
        assert events == []

    def test_single_point(self):
        data = [_make_ball_entry(0, 0.0, 0.0, 0.0, 0.0)]
        events = detect_kick_events(data, None, None)
        assert events == []
