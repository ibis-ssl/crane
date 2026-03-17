"""YAML設定ファイルエクスポート."""

from __future__ import annotations

import logging
import time
from pathlib import Path

import yaml

from .models import OptimizationResult

logger = logging.getLogger(__name__)

# デフォルトのボール物理パラメータ
_DEFAULT_BALL_PHYSICS = {
    "gravity": -9.81,
    "air_resistance": 0.0,
    "height_threshold": 0.05,
    "speed_threshold": 0.1,
    "stop_threshold": 0.05,
}


def _build_straight_kick_dict(
    result: OptimizationResult,
    kick_power_overrides: dict[str, float] | None,
) -> dict:
    """power_velocity_summary と kick_power_overrides をマージして straight_kick 辞書を返す."""
    straight_kick = {}
    for key, mean_vel in result.power_velocity_summary.items():
        if kick_power_overrides and key in kick_power_overrides:
            mean_vel = kick_power_overrides[key]
        power_val = int(key.replace("power_", "")) / 100.0
        sample_count = sum(
            1
            for k in result.kick_data
            if not k.is_chip_kick and abs(k.kick_power - power_val) < 0.05
        )
        straight_kick[key] = {"mean_velocity": mean_vel, "sample_count": sample_count}

    if kick_power_overrides:
        for key, mean_vel in kick_power_overrides.items():
            if key not in straight_kick:
                straight_kick[key] = {"mean_velocity": mean_vel, "sample_count": 0}

    return straight_kick


def export_calibration_yaml(
    result: OptimizationResult,
    output_path: str | Path,
    deceleration_override: float | None = None,
    kick_power_overrides: dict[str, float] | None = None,
) -> bool:
    """最適化結果を calibrated_ball_physics.yaml 形式で出力.

    C++ BallCalibrationNode の saveCalibrationResults に相当。
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    deceleration = (
        deceleration_override
        if deceleration_override is not None
        else result.global_deceleration
    )
    ball_physics = {**_DEFAULT_BALL_PHYSICS, "deceleration": deceleration}
    straight_kick = _build_straight_kick_dict(result, kick_power_overrides)

    now = int(time.time())
    doc = {
        "ball_physics_model": ball_physics,
        "kicker_power_mapping": {"straight_kick": straight_kick},
        "calibration_info": {
            "timestamp": now,
            "physics_rmse": result.global_rmse,
            "physics_r_squared": result.global_r_squared,
            "trajectories_analyzed": result.trajectories_analyzed,
            "trajectories_used": result.trajectories_used,
        },
    }

    try:
        with output_path.open("w") as f:
            f.write("# ボール物理モデル キャリブレーション結果\n")
            f.write(f"# 生成日時: {now}\n")
            f.write(f"# グローバル減速度: {deceleration:.4f} m/s²\n")
            f.write(f"# RMSE: {result.global_rmse:.6f}\n")
            f.write(f"# R²: {result.global_r_squared:.6f}\n")
            f.write(f"# 分析軌道数: {result.trajectories_analyzed}\n")
            f.write(f"# 有効軌道数: {result.trajectories_used}\n")
            f.write("\n")
            yaml.dump(
                doc, f, allow_unicode=True, default_flow_style=False, sort_keys=False
            )

        logger.info("YAMLエクスポート完了: %s", output_path)
        return True
    except Exception as e:
        logger.error("YAMLエクスポートエラー: %s", e)
        return False


def build_yaml_preview(
    result: OptimizationResult,
    deceleration_override: float | None = None,
    kick_power_overrides: dict[str, float] | None = None,
) -> str:
    """エクスポートされるYAMLの文字列プレビューを生成."""
    deceleration = (
        deceleration_override
        if deceleration_override is not None
        else result.global_deceleration
    )
    ball_physics = {**_DEFAULT_BALL_PHYSICS, "deceleration": deceleration}
    straight_kick = _build_straight_kick_dict(result, kick_power_overrides)

    doc = {
        "ball_physics_model": ball_physics,
        "kicker_power_mapping": {"straight_kick": straight_kick},
        "calibration_info": {
            "physics_rmse": result.global_rmse,
            "physics_r_squared": result.global_r_squared,
            "trajectories_analyzed": result.trajectories_analyzed,
            "trajectories_used": result.trajectories_used,
        },
    }
    return yaml.dump(doc, allow_unicode=True, default_flow_style=False, sort_keys=False)


def build_launch_array_preview(
    result: OptimizationResult,
    kick_power_overrides: dict[str, float] | None = None,
) -> dict[str, list]:
    """crane.launch.xml 用の配列形式プレビューを生成."""
    powers = []
    speeds = []
    for key, mean_vel in sorted(result.power_velocity_summary.items()):
        if kick_power_overrides and key in kick_power_overrides:
            mean_vel = kick_power_overrides[key]
        power_val = int(key.replace("power_", "")) / 100.0
        powers.append(round(power_val, 2))
        speeds.append(round(mean_vel, 4))

    return {
        "straight_kick_power_array": powers,
        "straight_kick_speed_array": speeds,
    }
