"""FastAPI バックエンド + ROS2ノード起動."""

from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Any

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles

from .data_loader import load_all_trajectories
from .models import (
    ExportRequest,
    LoadRequest,
    ManualParamsRequest,
    OptimizationConfig,
    OptimizationResult,
    OptimizeRequest,
    PredictRequest,
    TrajectoryData,
    TrajectoryInfo,
)
from .optimizer import compute_predicted_trajectories, run_optimization
from .yaml_exporter import (
    build_launch_array_preview,
    build_yaml_preview,
    export_calibration_yaml,
)

logger = logging.getLogger(__name__)

# アプリケーション状態
_state: dict[str, Any] = {
    "trajectories": [],
    "optimization_result": None,
    "current_deceleration": 0.7,
    "kick_power_overrides": {},
}

app = FastAPI(
    title="Ball Calibration UI",
    description="ボール物理モデルパラメータのインタラクティブ最適化WebUI",
    version="1.0.0",
)

# 静的ファイルディレクトリを解決
_STATIC_DIR = Path(__file__).parent / "static"
if _STATIC_DIR.exists():
    app.mount("/static", StaticFiles(directory=str(_STATIC_DIR)), name="static")


@app.get("/", response_class=HTMLResponse)
async def root() -> Any:
    """メインUIページを返す."""
    index_html = _STATIC_DIR / "index.html"
    if not index_html.exists():
        raise HTTPException(status_code=404, detail="index.html not found")
    return FileResponse(str(index_html))


@app.post("/api/load")
async def load_data(req: LoadRequest) -> dict:
    """JSONディレクトリからデータを読み込む."""
    directory = Path(req.directory_path)
    if not directory.exists():
        raise HTTPException(
            status_code=400, detail=f"ディレクトリが存在しません: {directory}"
        )

    trajectories = load_all_trajectories(directory)
    _state["trajectories"] = trajectories
    _state["trajectory_index"] = {t.event_id: t for t in trajectories}
    _state["optimization_result"] = None

    return {
        "loaded": len(trajectories),
        "directory": str(directory),
    }


@app.get("/api/trajectories")
async def get_trajectories() -> list[TrajectoryInfo]:
    """読み込み済み軌道の一覧を返す."""
    return [
        TrajectoryInfo(
            event_id=t.event_id,
            kick_power=t.kick_power,
            is_chip_kick=t.is_chip_kick,
            data_points=len(t.time_points),
            duration=t.duration,
            max_velocity=t.max_velocity,
        )
        for t in _state["trajectories"]
    ]


@app.get("/api/trajectory/{event_id}")
async def get_trajectory(event_id: int) -> dict:
    """個別軌道の詳細データを返す."""
    traj = _find_trajectory(event_id)
    return {
        "event_id": traj.event_id,
        "kick_power": traj.kick_power,
        "is_chip_kick": traj.is_chip_kick,
        "time_points": traj.time_points,
        "positions_x": traj.positions_x,
        "positions_y": traj.positions_y,
        "velocities": traj.velocities,
        "duration": traj.duration,
        "max_velocity": traj.max_velocity,
    }


@app.post("/api/optimize")
async def optimize(req: OptimizeRequest) -> OptimizationResult:
    """最適化を実行する."""
    trajectories: list[TrajectoryData] = _state["trajectories"]
    if not trajectories:
        raise HTTPException(status_code=400, detail="データが読み込まれていません")

    result = run_optimization(
        trajectories=trajectories,
        config=req.config,
        enabled_event_ids=req.enabled_event_ids,
        time_ranges={int(k): v for k, v in req.time_ranges.items()}
        if req.time_ranges
        else None,
    )

    _state["optimization_result"] = result
    _state["current_deceleration"] = result.global_deceleration

    return result


@app.post("/api/predict")
async def predict(req: PredictRequest) -> dict:
    """指定した減速度で予測軌道を計算する."""
    trajectories: list[TrajectoryData] = _state["trajectories"]
    if not trajectories:
        raise HTTPException(status_code=400, detail="データが読み込まれていません")

    config = OptimizationConfig()
    if req.event_ids is not None:
        id_set = set(req.event_ids)
        trajectories = [t for t in trajectories if t.event_id in id_set]

    predicted = compute_predicted_trajectories(trajectories, req.deceleration, config)
    _state["current_deceleration"] = req.deceleration

    return {
        "deceleration": req.deceleration,
        "trajectories": [p.model_dump() for p in predicted],
    }


@app.post("/api/export")
async def export_yaml(req: ExportRequest) -> dict:
    """最適化結果をYAMLにエクスポートする."""
    result: OptimizationResult | None = _state["optimization_result"]
    if result is None or not result.success:
        raise HTTPException(status_code=400, detail="最適化が完了していません")

    success = export_calibration_yaml(
        result=result,
        output_path=req.output_path,
        deceleration_override=req.deceleration_override,
        kick_power_overrides=req.kick_power_overrides,
    )

    if not success:
        raise HTTPException(status_code=500, detail="YAMLエクスポートに失敗しました")

    return {"success": True, "output_path": req.output_path}


@app.get("/api/export/preview")
async def export_preview() -> dict:
    """エクスポートされるYAMLのプレビューを返す."""
    result: OptimizationResult | None = _state["optimization_result"]
    if result is None or not result.success:
        return {"yaml": "", "launch_arrays": {}}

    overrides = _state.get("kick_power_overrides") or {}
    decel = _state.get("current_deceleration")

    yaml_text = build_yaml_preview(
        result,
        deceleration_override=decel if decel != result.global_deceleration else None,
        kick_power_overrides=overrides if overrides else None,
    )
    launch_arrays = build_launch_array_preview(
        result,
        kick_power_overrides=overrides if overrides else None,
    )

    return {
        "yaml": yaml_text,
        "launch_arrays": launch_arrays,
    }


@app.get("/api/current_params")
async def current_params() -> dict:
    """現在の最適化結果と手動調整値を返す."""
    result: OptimizationResult | None = _state["optimization_result"]
    return {
        "optimization_result": result.model_dump() if result else None,
        "current_deceleration": _state["current_deceleration"],
        "kick_power_overrides": _state.get("kick_power_overrides", {}),
    }


@app.put("/api/manual_params")
async def update_manual_params(req: ManualParamsRequest) -> dict:
    """手動調整パラメータを更新する."""
    if req.deceleration is not None:
        _state["current_deceleration"] = req.deceleration
    if req.kick_power_overrides is not None:
        _state["kick_power_overrides"] = req.kick_power_overrides
    return {"success": True}


def _find_trajectory(event_id: int) -> TrajectoryData:
    """event_id で軌道を検索する (O(1))."""
    t = _state.get("trajectory_index", {}).get(event_id)
    if t is None:
        raise HTTPException(
            status_code=404, detail=f"event_id={event_id} の軌道が見つかりません"
        )
    return t


def main() -> None:
    """ROS2エントリポイント."""
    import rclpy

    rclpy.init()
    node = rclpy.create_node("ball_calibration_ui")

    # ROS2パラメータ
    node.declare_parameter("port", 8095)
    node.declare_parameter("host", "0.0.0.0")
    port = node.get_parameter("port").get_parameter_value().integer_value
    host = node.get_parameter("host").get_parameter_value().string_value

    node.get_logger().info(f"Ball Calibration UI を起動します: http://{host}:{port}")

    # uvicorn をサブスレッドで起動
    def run_server() -> None:
        uvicorn.run(app, host=host, port=port, log_level="info")

    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
