"""data_loader.py の単体テスト."""

import json
from pathlib import Path

from crane_ball_calibration_ui.data_loader import (
    load_all_trajectories,
    load_trajectory_from_json,
)


def make_json_file(directory: Path, event_id: int, kick_power: float = 0.5) -> Path:
    """テスト用JSONファイルを生成する."""
    n = 30
    dt_ns = 50_000_000  # 50ms
    data = {
        "kick_info": {"kick_power": kick_power, "is_chip_kick": False},
        "data": {
            "timestamp_ns": [i * dt_ns for i in range(n)],
            "position": {
                "x": [i * 0.1 for i in range(n)],
                "y": [0.0] * n,
            },
            "ball_state": [1] * n,  # ROLLING
        },
    }
    path = directory / f"kick_event_visualization_{event_id}_data.json"
    path.write_text(json.dumps(data))
    return path


class TestLoadTrajectoryFromJson:
    def test_basic_load(self, tmp_path):
        path = make_json_file(tmp_path, event_id=42, kick_power=0.6)
        traj = load_trajectory_from_json(path)

        assert traj is not None
        assert traj.event_id == 42
        assert abs(traj.kick_power - 0.6) < 1e-6
        assert not traj.is_chip_kick
        assert len(traj.time_points) == 30
        assert traj.time_points[0] == 0.0  # 相対時間
        assert abs(traj.time_points[1] - 0.05) < 1e-6  # 50ms

    def test_rolling_only(self, tmp_path):
        """ROLLING (state=1) 以外のデータは除外される."""
        n = 20
        data = {
            "kick_info": {"kick_power": 0.5, "is_chip_kick": False},
            "data": {
                "timestamp_ns": list(range(n)),
                "position": {"x": list(range(n)), "y": [0.0] * n},
                "ball_state": [
                    0,
                    1,
                    1,
                    1,
                    0,
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,
                    0,
                ],
            },
        }
        path = tmp_path / "kick_event_visualization_1_data.json"
        path.write_text(json.dumps(data))

        traj = load_trajectory_from_json(path)
        # state=0 の3点は除外されるので 17点
        assert traj is not None
        assert len(traj.time_points) == 17

    def test_nonexistent_file(self):
        traj = load_trajectory_from_json("/nonexistent/path/file.json")
        assert traj is None

    def test_velocity_computed(self, tmp_path):
        """速度が差分計算されているか."""
        path = make_json_file(tmp_path, event_id=1)
        traj = load_trajectory_from_json(path)
        assert traj is not None
        assert len(traj.velocities) == len(traj.time_points)
        # 全速度が非負
        assert all(v >= 0 for v in traj.velocities)


class TestLoadAllTrajectories:
    def test_load_multiple_files(self, tmp_path):
        for i in range(3):
            make_json_file(tmp_path, event_id=i)
        trajectories = load_all_trajectories(tmp_path)
        assert len(trajectories) == 3

    def test_ignore_non_matching_files(self, tmp_path):
        make_json_file(tmp_path, event_id=1)
        # パターン外ファイルを作成
        (tmp_path / "other_file.json").write_text("{}")
        (tmp_path / "kick_event_1.json").write_text("{}")
        trajectories = load_all_trajectories(tmp_path)
        assert len(trajectories) == 1

    def test_nonexistent_directory(self):
        trajectories = load_all_trajectories("/nonexistent/directory")
        assert trajectories == []
