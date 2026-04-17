"""YAML設定ファイルエクスポート."""

from __future__ import annotations

import logging
import time

import yaml

from models import OptimizationResult

logger = logging.getLogger(__name__)

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


def _build_doc(
    result: OptimizationResult,
    deceleration_override: float | None,
    kick_power_overrides: dict[str, float] | None,
    include_timestamp: bool = False,
) -> tuple[float, dict]:
    """共通のYAML出力ドキュメントを構築する."""
    deceleration = (
        deceleration_override
        if deceleration_override is not None
        else result.global_deceleration
    )
    ball_physics = {**_DEFAULT_BALL_PHYSICS, "deceleration": deceleration}
    straight_kick = _build_straight_kick_dict(result, kick_power_overrides)

    calibration_info: dict = {
        "physics_rmse": result.global_rmse,
        "physics_r_squared": result.global_r_squared,
        "trajectories_analyzed": result.trajectories_analyzed,
        "trajectories_used": result.trajectories_used,
    }
    if result.aggregate_stats is not None:
        agg = result.aggregate_stats
        calibration_info["deceleration_ci_low"] = agg.ci_decel[0]
        calibration_info["deceleration_ci_high"] = agg.ci_decel[1]
        calibration_info["aggregation_method"] = agg.method
        calibration_info["inlier_trajectory_ratio"] = agg.inlier_trajectory_ratio
    if include_timestamp:
        calibration_info = {"timestamp": int(time.time()), **calibration_info}

    doc = {
        "ball_physics_model": ball_physics,
        "kicker_power_mapping": {"straight_kick": straight_kick},
        "calibration_info": calibration_info,
    }
    return deceleration, doc


def build_yaml_string(
    result: OptimizationResult,
    deceleration_override: float | None = None,
    kick_power_overrides: dict[str, float] | None = None,
) -> str:
    """最適化結果をYAML文字列として生成（ダウンロード用）."""
    deceleration, doc = _build_doc(
        result, deceleration_override, kick_power_overrides, include_timestamp=True
    )
    now = doc["calibration_info"]["timestamp"]

    header = (
        f"# ボール物理モデル キャリブレーション結果\n"
        f"# 生成日時: {now}\n"
        f"# グローバル減速度: {deceleration:.4f} m/s²\n"
        f"# RMSE: {result.global_rmse:.6f}\n"
        f"# R²: {result.global_r_squared:.6f}\n"
        f"# 分析軌道数: {result.trajectories_analyzed}\n"
        f"# 有効軌道数: {result.trajectories_used}\n\n"
    )
    return header + yaml.dump(
        doc, allow_unicode=True, default_flow_style=False, sort_keys=False
    )


def build_yaml_preview(
    result: OptimizationResult,
    deceleration_override: float | None = None,
    kick_power_overrides: dict[str, float] | None = None,
) -> str:
    """エクスポートされるYAMLの文字列プレビューを生成."""
    _, doc = _build_doc(result, deceleration_override, kick_power_overrides)
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

    return {"straight_kick_power_array": powers, "straight_kick_speed_array": speeds}
