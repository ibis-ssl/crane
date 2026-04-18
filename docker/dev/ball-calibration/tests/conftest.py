"""pytest 共通フィクスチャ."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

# /app 相当のパスを追加（Docker外でも動くよう）
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


def make_synthetic_trajectory(
    v0: float,
    deceleration: float,
    n_points: int = 40,
    dt: float = 0.05,
    noise_std: float = 0.02,
    outlier_ratio: float = 0.0,
    rng_seed: int = 42,
) -> tuple[np.ndarray, np.ndarray]:
    """合成転がり軌道を生成: v(t) = max(0, v0 - deceleration * t) + noise."""
    rng = np.random.default_rng(rng_seed)
    t = np.arange(n_points) * dt
    v_true = np.maximum(0.0, v0 - deceleration * t)
    noise = rng.normal(0.0, noise_std, size=n_points)
    v = v_true + noise

    # 外れ値を注入
    if outlier_ratio > 0.0:
        n_outliers = max(1, int(n_points * outlier_ratio))
        idx = rng.choice(n_points, size=n_outliers, replace=False)
        v[idx] += rng.uniform(1.0, 3.0, size=n_outliers) * rng.choice(
            [-1, 1], size=n_outliers
        )

    return t, np.clip(v, 0.0, None)
