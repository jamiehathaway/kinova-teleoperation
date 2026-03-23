"""Shared utilities for motor model fitting/validation scripts."""

import csv
import math
import os
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple


@dataclass
class JointModel:
    actuator_idx: int
    actuator_id: int
    torque_constant: float
    gear_ratio: float
    gear_efficiency: float
    bias: float
    coulomb_friction: float
    viscous_damping: float


def resolve_motor_model_path() -> str:
    env_path = os.getenv("KINOVA_TELEOP_MOTOR_MODEL")
    if env_path:
        return env_path

    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.abspath(os.path.join(here, "..", "config", "motor_model.csv"))
    return candidate


def load_motor_model_csv(path: str) -> List[JointModel]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Motor model file not found: {path}")

    models: List[JointModel] = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        required = [
            "actuator_idx",
            "actuator_id",
            "torque_constant",
            "gear_ratio",
            "gear_efficiency",
            "bias",
        ]
        if not reader.fieldnames:
            raise ValueError(f"Motor model CSV has no header: {path}")
        missing = [c for c in required if c not in reader.fieldnames]
        if missing:
            raise ValueError(f"Motor model CSV missing columns: {', '.join(missing)}")

        for row in reader:
            if not row:
                continue
            coulomb = float(row.get("coulomb_friction", 0.0) or 0.0)
            viscous = float(row.get("viscous_damping", 0.0) or 0.0)
            models.append(
                JointModel(
                    actuator_idx=int(float(row["actuator_idx"])),
                    actuator_id=int(float(row["actuator_id"])),
                    torque_constant=float(row["torque_constant"]),
                    gear_ratio=float(row["gear_ratio"]),
                    gear_efficiency=float(row["gear_efficiency"]),
                    bias=float(row["bias"]),
                    coulomb_friction=coulomb,
                    viscous_damping=viscous,
                )
            )

    if not models:
        raise ValueError(f"Motor model CSV is empty: {path}")
    return models


def require_columns(header: Iterable[str], columns: Iterable[str], csv_path: str) -> None:
    missing = [c for c in columns if c not in header]
    if missing:
        raise ValueError(f"Missing columns in {csv_path}: {', '.join(missing)}")


def fit_line(xs: Sequence[float], ys: Sequence[float]) -> Tuple[float, float, float]:
    n = len(xs)
    if n < 2:
        return float("nan"), float("nan"), float("nan")
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    var_x = sum((x - mean_x) ** 2 for x in xs)
    if var_x == 0.0:
        return float("nan"), float("nan"), float("nan")
    cov_xy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    a = cov_xy / var_x
    b = mean_y - a * mean_x

    ss_tot = sum((y - mean_y) ** 2 for y in ys)
    ss_res = sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else float("nan")
    return a, b, r2


def fit_line_full(xs: Sequence[float], ys: Sequence[float]) -> Tuple[float, float, float, float, float, float]:
    n = len(xs)
    if n < 2:
        return float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan")
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    var_x = sum((x - mean_x) ** 2 for x in xs)
    cov_xy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    slope = cov_xy / var_x if var_x > 0 else float("nan")
    intercept = mean_y - slope * mean_x if not math.isnan(slope) else float("nan")

    ss_tot = sum((y - mean_y) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else float("nan")

    errors = [x - y for x, y in zip(xs, ys)]
    bias = sum(errors) / n
    rmse = math.sqrt(sum(e * e for e in errors) / n)

    std_x = math.sqrt(var_x / n) if n > 0 else float("nan")
    std_y = math.sqrt(sum((y - mean_y) ** 2 for y in ys) / n) if n > 0 else float("nan")
    corr = cov_xy / (n * std_x * std_y) if std_x > 0 and std_y > 0 else float("nan")
    return slope, intercept, r2, rmse, bias, corr


def fit_drive_model(
    torque_nm: Sequence[float],
    current_a: Sequence[float],
    velocity_rad_s: Sequence[float],
) -> Tuple[float, float, float, float, float, float]:
    """Fit current = b + a*tau + c*sign(qdot) + d*qdot.

    Returns (a, b, c, d, r2, rmse).
    """
    try:
        import numpy as np
    except Exception as exc:
        raise RuntimeError("numpy is required for drive model fitting") from exc

    if len(torque_nm) < 2:
        return float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan")

    tau = np.asarray(torque_nm, dtype=float)
    curr = np.asarray(current_a, dtype=float)
    vel = np.asarray(velocity_rad_s, dtype=float)
    sign = np.sign(vel)
    X = np.column_stack([tau, np.ones_like(tau), sign, vel])
    coeffs, _, _, _ = np.linalg.lstsq(X, curr, rcond=None)
    a, b, c, d = [float(x) for x in coeffs]

    pred = X @ coeffs
    resid = curr - pred
    ss_tot = float(((curr - curr.mean()) ** 2).sum())
    ss_res = float((resid ** 2).sum())
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else float("nan")
    rmse = float((ss_res / len(curr)) ** 0.5)
    return a, b, c, d, r2, rmse


def torque_to_current(
    torque_nm: float,
    qdot_rad_s: float,
    model: JointModel,
    clamp: Optional[float] = None,
) -> float:
    gain = model.torque_constant * model.gear_ratio * model.gear_efficiency
    if abs(gain) < 1e-9:
        return 0.0

    friction = 0.0
    if qdot_rad_s > 0.0:
        friction = model.coulomb_friction
    elif qdot_rad_s < 0.0:
        friction = -model.coulomb_friction
    friction += model.viscous_damping * qdot_rad_s

    current = (torque_nm + friction) / gain + model.bias
    if clamp is not None:
        current = max(-clamp, min(clamp, current))
    return current
