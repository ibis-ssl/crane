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

from ssl_log_extractor import extract_ball_timeline_and_trajectories
from models import (
    AddTrajectoryRequest,
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
from yaml_exporter import build_yaml_preview, build_yaml_string

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

_state: dict[str, Any] = {
    "trajectories": [],
    "optimization_result": None,
    "current_deceleration": 0.7,
    "raw_ball_data": [],
    "timeline_origin_ns": None,
}

WEB_DIR = Path(__file__).resolve().parent / "web"


def _find_trajectory(event_id: int) -> TrajectoryData:
    for t in _state["trajectories"]:
        if t.event_id == event_id:
            return t
    raise HTTPException(
        status_code=404, detail=f"event_id={event_id} の軌道が見つかりません"
    )


def _store_trajectories(
    trajectories: list[TrajectoryData],
    filename: str,
    path: str | None = None,
    ball_data: list[tuple] | None = None,
) -> dict:
    _state["trajectories"] = trajectories
    _state["optimization_result"] = None
    _state["current_deceleration"] = 0.7
    if path:
        _state["current_path"] = path
    if ball_data is not None:
        _state["raw_ball_data"] = ball_data
        _state["timeline_origin_ns"] = ball_data[0][0] if ball_data else None
    return {"loaded": len(trajectories), "filename": filename}


app = FastAPI(title="Ball Calibration UI")

shared_dir = WEB_DIR / "shared"
if shared_dir.exists():
    app.mount("/shared", StaticFiles(directory=str(shared_dir)), name="shared")

fonts_dir = Path(os.environ.get("FONTS_DIR", "/app/fonts"))
if fonts_dir.is_dir():
    app.mount("/fonts", StaticFiles(directory=str(fonts_dir)), name="fonts")

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


@app.get("/api/file_dialog")
async def file_dialog(start_path: str = "/home") -> dict:
    """zenity でシステムネイティブのファイル選択ダイアログを開き、選択パスを返す."""
    import asyncio

    start = Path(start_path)
    filename_arg = f"--filename={start}/" if start.is_dir() else f"--filename={start}"

    try:
        proc = await asyncio.create_subprocess_exec(
            "zenity",
            "--file-selection",
            "--title=SSL ログを選択",
            "--file-filter=SSL Log (*.log.gz) | *.log.gz",
            filename_arg,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=120)
    except FileNotFoundError:
        raise HTTPException(
            status_code=500, detail="zenity がインストールされていません"
        )
    except asyncio.TimeoutError:
        proc.kill()
        raise HTTPException(status_code=408, detail="タイムアウト")

    if proc.returncode != 0:
        raise HTTPException(status_code=400, detail="キャンセルされました")

    selected = stdout.decode().strip()
    return {"path": selected}


@app.get("/api/browse")
async def browse(path: str = "/") -> dict:
    """サーバー上のディレクトリを一覧する."""
    base = Path(path).resolve()
    if not base.exists() or not base.is_dir():
        raise HTTPException(
            status_code=400, detail=f"ディレクトリが見つかりません: {path}"
        )

    dirs, files = [], []
    try:
        for entry in sorted(base.iterdir()):
            if entry.name.startswith("."):
                continue
            if entry.is_dir():
                dirs.append({"name": entry.name, "path": str(entry)})
            elif entry.name.endswith(".log.gz"):
                files.append(
                    {
                        "name": entry.name,
                        "path": str(entry),
                        "size": entry.stat().st_size,
                    }
                )
    except PermissionError:
        pass

    parent = str(base.parent) if base != base.parent else None
    return {"current": str(base), "parent": parent, "dirs": dirs, "files": files}


@app.post("/api/load_path")
async def load_path(req: LoadPathRequest) -> dict:
    """SSL ログファイルパスを直接指定して軌道データを読み込む."""
    path = Path(req.path)
    if not path.exists():
        raise HTTPException(status_code=400, detail=f"ファイルが見つかりません: {path}")
    if not path.name.endswith(".log.gz"):
        raise HTTPException(status_code=400, detail="対応ファイル形式: .log.gz")

    trajectories, ball_data = extract_ball_timeline_and_trajectories(path)
    return _store_trajectories(trajectories, path.name, str(path), ball_data)


@app.post("/api/upload")
async def upload_log(file: UploadFile) -> dict:
    """SSL ログファイルをアップロードして軌道データを抽出する."""
    filename = file.filename or "upload"
    if not filename.endswith(".log.gz"):
        raise HTTPException(status_code=400, detail="対応ファイル形式: .log.gz")

    with tempfile.NamedTemporaryFile(suffix=".log.gz", delete=False) as tmp:
        tmp_path = Path(tmp.name)
        shutil.copyfileobj(file.file, tmp)

    try:
        trajectories, ball_data = extract_ball_timeline_and_trajectories(tmp_path)
    finally:
        tmp_path.unlink(missing_ok=True)

    return _store_trajectories(trajectories, filename, ball_data=ball_data)


@app.get("/api/trajectories")
async def get_trajectories() -> list[TrajectoryInfo]:
    return [
        TrajectoryInfo(
            event_id=t.event_id,
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
    if result.global_deceleration > 0.0:
        _state["current_deceleration"] = result.global_deceleration
    return result


@app.get("/api/timeline")
async def get_timeline(max_points: int = 4000) -> dict:
    """ログ全体のボール速度時系列を返す（表示用にダウンサンプリング）."""
    import math as _math

    raw = _state.get("raw_ball_data") or []
    origin_ns = _state.get("timeline_origin_ns")
    if not raw or origin_ns is None:
        return {"time_points": [], "velocities": [], "total_duration": 0.0}

    step = max(1, len(raw) // max_points)

    time_points: list[float] = []
    velocities: list[float] = []
    for i in range(0, len(raw), step):
        ts, _x, _y, _z, vx, vy, *_ = raw[i]
        time_points.append((ts - origin_ns) * 1e-9)
        velocities.append(_math.hypot(vx, vy))

    total_duration = (raw[-1][0] - origin_ns) * 1e-9
    return {
        "time_points": time_points,
        "velocities": velocities,
        "total_duration": total_duration,
        "total_samples": len(raw),
        "downsampled_to": len(time_points),
    }


@app.post("/api/add_trajectory")
async def add_trajectory(req: AddTrajectoryRequest) -> dict:
    """タイムライン上で手動選択した時刻範囲を軌道として追加する."""
    import math as _math

    raw = _state.get("raw_ball_data") or []
    origin_ns = _state.get("timeline_origin_ns")
    if not raw or origin_ns is None:
        raise HTTPException(
            status_code=400, detail="タイムラインが読み込まれていません"
        )
    if req.end_time <= req.start_time:
        raise HTTPException(
            status_code=400, detail="end_time は start_time より大きくしてください"
        )

    start_ns = int(origin_ns + req.start_time * 1e9)
    end_ns = int(origin_ns + req.end_time * 1e9)

    sub = [
        (ts, x, y, _math.hypot(vx, vy))
        for (ts, x, y, _z, vx, vy, *_) in raw
        if start_ns <= ts <= end_ns
    ]
    if len(sub) < 3:
        raise HTTPException(
            status_code=400, detail=f"選択範囲の点数が不足しています: {len(sub)} 点"
        )

    existing_ids = [t.event_id for t in _state["trajectories"]]
    new_id = max(existing_ids, default=-1) + 1

    t0_ns = sub[0][0]
    time_points = [(ts - t0_ns) * 1e-9 for ts, *_ in sub]
    positions_x = [x for _, x, _, _ in sub]
    positions_y = [y for _, _, y, _ in sub]
    velocities = [v for *_, v in sub]

    new_traj = TrajectoryData(
        event_id=new_id,
        time_points=time_points,
        positions_x=positions_x,
        positions_y=positions_y,
        velocities=velocities,
    )
    _state["trajectories"].append(new_traj)
    return {
        "event_id": new_id,
        "data_points": len(time_points),
        "duration": new_traj.duration,
        "max_velocity": new_traj.max_velocity,
    }


@app.delete("/api/trajectory/{event_id}")
async def delete_trajectory(event_id: int) -> dict:
    """軌道を削除する."""
    trajectories: list[TrajectoryData] = _state["trajectories"]
    before = len(trajectories)
    _state["trajectories"] = [t for t in trajectories if t.event_id != event_id]
    if len(_state["trajectories"]) == before:
        raise HTTPException(
            status_code=404, detail=f"event_id={event_id} が見つかりません"
        )
    return {"deleted": event_id, "remaining": len(_state["trajectories"])}


@app.get("/api/bootstrap/{event_id}")
async def get_bootstrap(event_id: int, n_boot: int = 300) -> dict:
    """指定軌道のブートストラップ分布を返す (UI ヒストグラム用)."""
    from robust_fit import bootstrap_ci, pick_fit_fn
    import numpy as np

    result: OptimizationResult | None = _state["optimization_result"]
    if result is None:
        raise HTTPException(status_code=400, detail="最適化が完了していません")

    traj = _find_trajectory(event_id)
    config = OptimizationConfig()

    fit_fn = pick_fit_fn(config.algorithm, config.physics_model)
    t_arr = np.array(traj.time_points)
    v_arr = np.array(traj.velocities)
    v0_samples, decel_samples = bootstrap_ci(t_arr, v_arr, fit_fn, n_boot=n_boot)

    return {
        "event_id": event_id,
        "v0_samples": v0_samples,
        "decel_samples": decel_samples,
    }


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
    """キャリブレーション結果を YAML ファイルとしてダウンロードする."""
    result: OptimizationResult | None = _state["optimization_result"]
    if result is None or not result.success:
        raise HTTPException(status_code=400, detail="最適化が完了していません")

    decel = _state.get("current_deceleration")
    yaml_text = build_yaml_string(
        result,
        deceleration_override=decel if decel != result.global_deceleration else None,
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
        return {"yaml": ""}

    decel = _state.get("current_deceleration")
    return {
        "yaml": build_yaml_preview(
            result,
            deceleration_override=decel
            if decel != result.global_deceleration
            else None,
        ),
    }


@app.get("/api/current_params")
async def current_params() -> dict:
    result: OptimizationResult | None = _state["optimization_result"]
    return {
        "optimization_result": result.model_dump() if result else None,
        "current_deceleration": _state["current_deceleration"],
    }


@app.put("/api/manual_params")
async def update_manual_params(req: ManualParamsRequest) -> dict:
    if req.deceleration is not None:
        _state["current_deceleration"] = req.deceleration
    return {"success": True}


if __name__ == "__main__":
    port = int(os.environ.get("HTTP_PORT", 8090))
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")
