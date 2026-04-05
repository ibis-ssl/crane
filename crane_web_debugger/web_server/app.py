"""Unified HTTP server for crane_web_debugger and ball calibration UI."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import uvicorn
from fastapi import APIRouter, FastAPI, HTTPException
from fastapi.responses import FileResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

from ball_calibration.data_loader import load_all_trajectories
from ball_calibration.models import (
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
from ball_calibration.optimizer import compute_predicted_trajectories, run_optimization
from ball_calibration.yaml_exporter import (
    build_launch_array_preview,
    build_yaml_preview,
    export_calibration_yaml,
)

_state: dict[str, Any] = {
    "trajectories": [],
    "trajectory_index": {},
    "optimization_result": None,
    "current_deceleration": 0.7,
    "kick_power_overrides": {},
}


def _find_trajectory(event_id: int) -> TrajectoryData:
    t = _state.get("trajectory_index", {}).get(event_id)
    if t is None:
        raise HTTPException(
            status_code=404, detail=f"event_id={event_id} の軌道が見つかりません"
        )
    return t


def create_app(web_root: Path) -> FastAPI:
    app = FastAPI(title="Crane Web Debugger Unified HTTP")

    ball_ui_dir = web_root / "ball-calibration"
    ball_index = ball_ui_dir / "index.html"
    ball_static = ball_ui_dir / "static"

    if ball_static.exists():
        app.mount(
            "/ball-calibration/static",
            StaticFiles(directory=str(ball_static)),
            name="ball-calibration-static",
        )

    @app.get("/ball-calibration")
    async def ball_calibration_redirect() -> RedirectResponse:
        return RedirectResponse(url="/ball-calibration/")

    @app.get("/ball-calibration/")
    async def ball_calibration_root() -> FileResponse:
        if not ball_index.exists():
            raise HTTPException(
                status_code=404, detail="ball-calibration index not found"
            )
        return FileResponse(str(ball_index))

    api = APIRouter(prefix="/ball-calibration/api")

    @api.post("/load")
    async def load_data(req: LoadRequest) -> dict:
        directory = Path(req.directory_path)
        if not directory.exists():
            raise HTTPException(
                status_code=400, detail=f"ディレクトリが存在しません: {directory}"
            )

        trajectories = load_all_trajectories(directory)
        _state["trajectories"] = trajectories
        _state["trajectory_index"] = {t.event_id: t for t in trajectories}
        _state["optimization_result"] = None
        _state["kick_power_overrides"] = {}
        _state["current_deceleration"] = 0.7

        return {
            "loaded": len(trajectories),
            "directory": str(directory),
        }

    @api.get("/trajectories")
    async def get_trajectories() -> list[TrajectoryInfo]:
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

    @api.get("/trajectory/{event_id}")
    async def get_trajectory(event_id: int) -> dict:
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

    @api.post("/optimize")
    async def optimize(req: OptimizeRequest) -> OptimizationResult:
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

    @api.post("/predict")
    async def predict(req: PredictRequest) -> dict:
        trajectories: list[TrajectoryData] = _state["trajectories"]
        if not trajectories:
            raise HTTPException(status_code=400, detail="データが読み込まれていません")

        config = OptimizationConfig()
        if req.event_ids is not None:
            id_set = set(req.event_ids)
            trajectories = [t for t in trajectories if t.event_id in id_set]

        predicted = compute_predicted_trajectories(
            trajectories, req.deceleration, config
        )
        _state["current_deceleration"] = req.deceleration

        return {
            "deceleration": req.deceleration,
            "trajectories": [p.model_dump() for p in predicted],
        }

    @api.post("/export")
    async def export_yaml(req: ExportRequest) -> dict:
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
            raise HTTPException(
                status_code=500, detail="YAMLエクスポートに失敗しました"
            )

        return {"success": True, "output_path": req.output_path}

    @api.get("/export/preview")
    async def export_preview() -> dict:
        result: OptimizationResult | None = _state["optimization_result"]
        if result is None or not result.success:
            return {"yaml": "", "launch_arrays": {}}

        overrides = _state.get("kick_power_overrides") or {}
        decel = _state.get("current_deceleration")

        yaml_text = build_yaml_preview(
            result,
            deceleration_override=decel
            if decel != result.global_deceleration
            else None,
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

    @api.get("/current_params")
    async def current_params() -> dict:
        result: OptimizationResult | None = _state["optimization_result"]
        return {
            "optimization_result": result.model_dump() if result else None,
            "current_deceleration": _state["current_deceleration"],
            "kick_power_overrides": _state.get("kick_power_overrides", {}),
        }

    @api.put("/manual_params")
    async def update_manual_params(req: ManualParamsRequest) -> dict:
        if req.deceleration is not None:
            _state["current_deceleration"] = req.deceleration
        if req.kick_power_overrides is not None:
            _state["kick_power_overrides"] = req.kick_power_overrides
        return {"success": True}

    app.include_router(api)

    # Existing crane_web_debugger static site (portal, viewer, telemetry, ...)
    app.mount("/", StaticFiles(directory=str(web_root), html=True), name="web-root")
    return app


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Unified crane web debugger HTTP server"
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--web-root", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    app = create_app(args.web_root)
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
