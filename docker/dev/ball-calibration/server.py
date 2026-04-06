"""Ball Calibration UI - 独立FastAPIサーバー."""

from __future__ import annotations

import io
import logging
import os
import shutil
import tempfile
from pathlib import Path
from typing import Any

import uvicorn
from fastapi import FastAPI, HTTPException, UploadFile
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from mcap_extractor import extract_trajectories_from_mcap
from models import (
    LoadPathRequest,
    ManualParamsRequest,
    OptimizationConfig,
    OptimizationResult,
    OptimizeRequest,
    PredictRequest,
    TrajectoryData,
    TrajectoryInfo,
)
from optimizer import compute_predicted_trajectories, run_optimization
from yaml_exporter import (
    build_launch_array_preview,
    build_yaml_preview,
    build_yaml_string,
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

_state: dict[str, Any] = {
    "trajectories": [],
    "optimization_result": None,
    "current_deceleration": 0.7,
    "kick_power_overrides": {},
}

WEB_DIR = Path(__file__).resolve().parent / "web"


def _find_trajectory(event_id: int) -> TrajectoryData:
    for t in _state["trajectories"]:
        if t.event_id == event_id:
            return t
    raise HTTPException(
        status_code=404, detail=f"event_id={event_id} の軌道が見つかりません"
    )


def _store_trajectories(trajectories: list[TrajectoryData], filename: str) -> dict:
    _state["trajectories"] = trajectories
    _state["optimization_result"] = None
    _state["kick_power_overrides"] = {}
    _state["current_deceleration"] = 0.7
    return {"loaded": len(trajectories), "filename": filename}


app = FastAPI(title="Ball Calibration UI")

static_dir = WEB_DIR / "static"
if static_dir.exists():
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")


@app.get("/")
async def root() -> FileResponse:
    index = WEB_DIR / "index.html"
    if not index.exists():
        raise HTTPException(status_code=404, detail="index.html not found")
    return FileResponse(str(index))


@app.get("/healthz")
async def healthz() -> dict:
    return {"ok": True, "service": "ball-calibration"}


@app.post("/api/load_path")
async def load_path(req: LoadPathRequest) -> dict:
    """サーバー上のmcapファイルパスを直接指定して軌道データを読み込む."""
    path = Path(req.path)
    if not path.exists():
        raise HTTPException(status_code=400, detail=f"ファイルが見つかりません: {path}")
    if path.suffix.lower() not in {".mcap", ".db3"}:
        raise HTTPException(
            status_code=400, detail="対応ファイル形式: .mcap または .db3"
        )

    return _store_trajectories(extract_trajectories_from_mcap(path), path.name)


@app.post("/api/upload")
async def upload_mcap(file: UploadFile) -> dict:
    """mcapファイルをアップロードして軌道データを抽出する."""
    filename = file.filename or "upload"
    suffix = Path(filename).suffix.lower()
    if suffix not in {".mcap", ".db3"}:
        raise HTTPException(
            status_code=400, detail="対応ファイル形式: .mcap または .db3"
        )

    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as tmp:
        tmp_path = Path(tmp.name)
        shutil.copyfileobj(file.file, tmp)

    try:
        trajectories = extract_trajectories_from_mcap(tmp_path)
    finally:
        tmp_path.unlink(missing_ok=True)

    return _store_trajectories(trajectories, filename)


@app.get("/api/trajectories")
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


@app.get("/api/trajectory/{event_id}")
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


@app.post("/api/optimize")
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


@app.post("/api/predict")
async def predict(req: PredictRequest) -> dict:
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


@app.get("/api/export/download")
async def export_download() -> StreamingResponse:
    """キャリブレーション結果をYAMLファイルとしてダウンロードする."""
    result: OptimizationResult | None = _state["optimization_result"]
    if result is None or not result.success:
        raise HTTPException(status_code=400, detail="最適化が完了していません")

    overrides = _state["kick_power_overrides"] or {}
    decel = _state.get("current_deceleration")

    yaml_text = build_yaml_string(
        result,
        deceleration_override=decel if decel != result.global_deceleration else None,
        kick_power_overrides=overrides if overrides else None,
    )

    return StreamingResponse(
        io.BytesIO(yaml_text.encode("utf-8")),
        media_type="application/x-yaml",
        headers={
            "Content-Disposition": "attachment; filename=calibrated_ball_physics.yaml"
        },
    )


@app.get("/api/export/preview")
async def export_preview() -> dict:
    result: OptimizationResult | None = _state["optimization_result"]
    if result is None or not result.success:
        return {"yaml": "", "launch_arrays": {}}

    overrides = _state["kick_power_overrides"] or {}
    decel = _state.get("current_deceleration")

    return {
        "yaml": build_yaml_preview(
            result,
            deceleration_override=decel
            if decel != result.global_deceleration
            else None,
            kick_power_overrides=overrides if overrides else None,
        ),
        "launch_arrays": build_launch_array_preview(
            result,
            kick_power_overrides=overrides if overrides else None,
        ),
    }


@app.get("/api/current_params")
async def current_params() -> dict:
    result: OptimizationResult | None = _state["optimization_result"]
    return {
        "optimization_result": result.model_dump() if result else None,
        "current_deceleration": _state["current_deceleration"],
        "kick_power_overrides": _state["kick_power_overrides"],
    }


@app.put("/api/manual_params")
async def update_manual_params(req: ManualParamsRequest) -> dict:
    if req.deceleration is not None:
        _state["current_deceleration"] = req.deceleration
    if req.kick_power_overrides is not None:
        _state["kick_power_overrides"] = req.kick_power_overrides
    return {"success": True}


if __name__ == "__main__":
    port = int(os.environ.get("HTTP_PORT", 8090))
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")
