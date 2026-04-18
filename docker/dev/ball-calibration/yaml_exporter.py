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


def _build_doc(
    result: OptimizationResult,
    deceleration_override: float | None,
    include_timestamp: bool = False,
) -> tuple[float, dict]:
    """共通の YAML 出力ドキュメントを構築する."""
    deceleration = (
        deceleration_override
        if deceleration_override is not None
        else result.global_deceleration
    )
    ball_physics = {**_DEFAULT_BALL_PHYSICS, "deceleration": deceleration}

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
        "calibration_info": calibration_info,
    }
    return deceleration, doc


def build_yaml_string(
    result: OptimizationResult,
    deceleration_override: float | None = None,
) -> str:
    """最適化結果を YAML 文字列として生成（ダウンロード用）."""
    deceleration, doc = _build_doc(
        result, deceleration_override, include_timestamp=True
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
) -> str:
    """エクスポートされる YAML の文字列プレビューを生成."""
    _, doc = _build_doc(result, deceleration_override)
    return yaml.dump(doc, allow_unicode=True, default_flow_style=False, sort_keys=False)
