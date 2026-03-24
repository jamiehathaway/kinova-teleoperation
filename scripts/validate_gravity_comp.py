#!/usr/bin/env python3
"""Validate model-based compensation against torque_control_log CSVs.

Modes:
- gravity: gravity-only model on low-velocity samples (default behavior).
- full: full dynamics (gravity + coriolis + inertia) using Pinocchio on all samples.

In both modes, model torque is mapped to current using the same motor model as
torque_control_test.cpp, and compared against measured actuator current.
"""

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

from gravity_model_backend import build_gravity_model
from motor_model_utils import fit_line_full, load_motor_model_csv, require_columns, resolve_motor_model_path


DEG2RAD = math.pi / 180.0
JOINTS = [1, 2, 3, 4, 5, 6]
DEFAULT_URDF = "src/ros2_kortex/kortex_description/robots/gen3_lite.urdf"
DEFAULT_LOGS = [
    "log/torque_control_log_1.csv",
    "log/torque_control_log_2.csv",
]
DEFAULT_ROOT_LINK = "base_link"
DEFAULT_TIP_LINK = "tool_frame"
MAX_ABS_CURRENT_A = 5.0


@dataclass
class FitStats:
    joint: int
    n: int
    slope: float
    intercept: float
    r2: float
    rmse: float
    bias: float
    corr: float


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Validate gravity compensation model using torque_control logs."
    )
    p.add_argument(
        "--csv",
        action="append",
        default=None,
        help="CSV path (repeatable). Defaults to torque_control_log_1/2.csv.",
    )
    p.add_argument(
        "--urdf",
        default=DEFAULT_URDF,
        help=f"URDF path (default: {DEFAULT_URDF})",
    )
    p.add_argument(
        "--root-link",
        default=DEFAULT_ROOT_LINK,
        help=f"Root link for KDL chain (default: {DEFAULT_ROOT_LINK})",
    )
    p.add_argument(
        "--tip-link",
        default=DEFAULT_TIP_LINK,
        help=f"Tip link for KDL chain (default: {DEFAULT_TIP_LINK})",
    )
    p.add_argument(
        "--backend",
        choices=["auto", "kdl", "pinocchio"],
        default="pinocchio",
        help="Gravity backend to use (default: pinocchio).",
    )
    p.add_argument(
        "--mode",
        choices=["gravity", "full"],
        default="gravity",
        help="Model mode: gravity-only (filtered) or full dynamics (Pinocchio).",
    )
    p.add_argument(
        "--vel-threshold-deg",
        type=float,
        default=1.0,
        help="Max abs joint velocity (deg/s) to keep sample.",
    )
    p.add_argument(
        "--plots-dir",
        default="log",
        help="Directory for per-joint plot PNGs (default: log).",
    )
    p.add_argument(
        "--plot-grid",
        default="log/gravity_sanity_grid.png",
        help="Optional combined grid plot path (default: log/gravity_sanity_grid.png).",
    )
    p.add_argument(
        "--summary-csv",
        default="log/gravity_sanity_summary.csv",
        help="Summary CSV output path.",
    )
    p.add_argument(
        "--summary-json",
        default="",
        help="Optional summary JSON output path.",
    )
    p.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip plotting (still writes summary metrics).",
    )
    p.add_argument(
        "--motor-model",
        default="",
        help="Path to motor_model.csv (default: resolved from scripts/config).",
    )
    return p.parse_args()




def load_rows(paths: Sequence[str], vel_thresh_deg: Optional[float]) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    for path in paths:
        if not os.path.exists(path):
            raise SystemExit(f"CSV file not found: {path}")
        with open(path, "r", newline="") as f:
            reader = csv.DictReader(f)
            header = reader.fieldnames or []
            required = ["timestamp_ns", "sample_idx"]
            for j in JOINTS:
                required.append(f"actuator_{j}_position")
                required.append(f"actuator_{j}_velocity")
                required.append(f"actuator_{j}_current")
                required.append(f"actuator_{j}_torque")
            try:
                require_columns(header, required, path)
            except ValueError as exc:
                raise SystemExit(str(exc)) from exc

            for row in reader:
                try:
                    max_abs_vel = max(
                        abs(float(row[f"actuator_{j}_velocity"])) for j in JOINTS
                    )
                except ValueError:
                    continue
                if vel_thresh_deg is not None and max_abs_vel > vel_thresh_deg:
                    continue

                parsed: Dict[str, float] = {
                    "timestamp_ns": float(row["timestamp_ns"]),
                    "sample_idx": float(row["sample_idx"]),
                }
                for j in JOINTS:
                    parsed[f"pos_{j}"] = float(row[f"actuator_{j}_position"]) * DEG2RAD
                    parsed[f"vel_{j}"] = float(row[f"actuator_{j}_velocity"]) * DEG2RAD
                    parsed[f"current_{j}"] = float(row[f"actuator_{j}_current"])
                    parsed[f"torque_{j}"] = float(row[f"actuator_{j}_torque"])
                rows.append(parsed)
    return rows


def torque_to_current(torque_nm: float, qdot_rad_s: float, model) -> float:
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
    return max(-MAX_ABS_CURRENT_A, min(MAX_ABS_CURRENT_A, current))


def compute_model(rows: List[Dict[str, float]], model, joint_models) -> None:
    model_by_idx = {m.actuator_idx: m for m in joint_models}
    missing = [j for j in range(len(JOINTS)) if j not in model_by_idx]
    if missing:
        raise SystemExit(f"Motor model missing actuator_idx entries: {missing}")

    for row in rows:
        q = [row[f"pos_{j}"] for j in JOINTS]
        tau = model.compute(q)
        for j, val in zip(JOINTS, tau):
            row[f"tau_model_{j}"] = val
            model_entry = model_by_idx[j - 1]
            row[f"current_model_{j}"] = torque_to_current(
                val, row[f"vel_{j}"], model_entry
            )


def compute_full_dynamics(rows: List[Dict[str, float]], urdf_path: str, joint_models) -> None:
    try:
        import pinocchio as pin  # type: ignore
        import numpy as np  # type: ignore
    except Exception as exc:
        raise SystemExit("Pinocchio is required for --mode full.") from exc

    joint_names = [f"joint_{j}" for j in JOINTS]
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()

    model_by_idx = {m.actuator_idx: m for m in joint_models}
    missing = [j for j in range(len(JOINTS)) if j not in model_by_idx]
    if missing:
        raise SystemExit(f"Motor model missing actuator_idx entries: {missing}")

    rows.sort(key=lambda r: r["timestamp_ns"])
    t = [row["timestamp_ns"] * 1e-9 for row in rows]
    n = len(rows)

    q_mat = np.array([[row[f"pos_{j}"] for j in JOINTS] for row in rows], dtype=float)
    v_mat = np.array([[row[f"vel_{j}"] for j in JOINTS] for row in rows], dtype=float)
    a_mat = np.zeros_like(v_mat)

    for i in range(n):
        if n < 2:
            break
        if i == 0:
            dt = t[1] - t[0]
            a_mat[i] = (v_mat[1] - v_mat[0]) / dt if dt > 0 else 0.0
        elif i == n - 1:
            dt = t[-1] - t[-2]
            a_mat[i] = (v_mat[-1] - v_mat[-2]) / dt if dt > 0 else 0.0
        else:
            dt = t[i + 1] - t[i - 1]
            a_mat[i] = (v_mat[i + 1] - v_mat[i - 1]) / dt if dt > 0 else 0.0

    for i, row in enumerate(rows):
        q = np.zeros(model.nq)
        v = np.zeros(model.nv)
        a = np.zeros(model.nv)
        for j_idx, name in enumerate(joint_names):
            jid = model.getJointId(name)
            if jid == 0:
                raise SystemExit(f"Joint '{name}' not found in URDF model.")
            q_idx = model.joints[jid].idx_q
            v_idx = model.joints[jid].idx_v
            q[q_idx] = q_mat[i, j_idx]
            v[v_idx] = v_mat[i, j_idx]
            a[v_idx] = a_mat[i, j_idx]

        tau = pin.rnea(model, data, q, v, a)
        for j_idx, name in enumerate(joint_names):
            jid = model.getJointId(name)
            v_idx = model.joints[jid].idx_v
            tau_j = float(tau[v_idx])
            row[f"tau_model_{j_idx + 1}"] = tau_j
            model_entry = model_by_idx[j_idx]
            row[f"current_model_{j_idx + 1}"] = torque_to_current(
                tau_j, row[f"vel_{j_idx + 1}"], model_entry
            )


def write_summary(stats: List[FitStats], csv_path: str, json_path: str) -> None:
    os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["joint", "n", "slope", "intercept", "r2", "rmse", "bias", "corr"])
        for s in stats:
            w.writerow([s.joint, s.n, s.slope, s.intercept, s.r2, s.rmse, s.bias, s.corr])

    if json_path:
        with open(json_path, "w", newline="") as f:
            json.dump([s.__dict__ for s in stats], f, indent=2)


def plot_per_joint(rows: List[Dict[str, float]], stats: List[FitStats], out_dir: str, grid_path: str) -> None:
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception as exc:
        raise SystemExit(
            "matplotlib is required for plotting. Install it in the devcontainer or use --no-plots."
        ) from exc

    os.makedirs(out_dir, exist_ok=True)

    # Per-joint plots
    for s in stats:
        xs = [row[f"current_model_{s.joint}"] for row in rows]
        ys = [row[f"current_{s.joint}"] for row in rows]
        fig, ax = plt.subplots(figsize=(5, 4), constrained_layout=True)
        ax.scatter(xs, ys, s=10, alpha=0.6)
        if not math.isnan(s.slope):
            x_min, x_max = min(xs), max(xs)
            ax.plot([x_min, x_max], [x_min, x_max], "k--", linewidth=1, label="1:1")
            ax.plot(
                [x_min, x_max],
                [s.slope * x_min + s.intercept, s.slope * x_max + s.intercept],
                "r-",
                linewidth=2,
                label="fit",
            )
        ax.set_title(f"Joint {s.joint} (R^2={s.r2:.3f})")
        ax.set_xlabel("gravity-comp current model (A)")
        ax.set_ylabel("measured actuator current (A)")
        ax.legend(loc="best", fontsize=8)
        fig.savefig(os.path.join(out_dir, f"gravity_sanity_joint{s.joint}.png"), dpi=150)
        plt.close(fig)

    # Grid plot
    if grid_path:
        fig, axes = plt.subplots(2, 3, figsize=(12, 6), constrained_layout=True)
        axes = axes.flatten()
        for idx, s in enumerate(stats):
            ax = axes[idx]
            xs = [row[f"current_model_{s.joint}"] for row in rows]
            ys = [row[f"current_{s.joint}"] for row in rows]
            ax.scatter(xs, ys, s=8, alpha=0.6)
            if not math.isnan(s.slope):
                x_min, x_max = min(xs), max(xs)
                ax.plot([x_min, x_max], [x_min, x_max], "k--", linewidth=1)
                ax.plot(
                    [x_min, x_max],
                    [s.slope * x_min + s.intercept, s.slope * x_max + s.intercept],
                    "r-",
                    linewidth=2,
                )
            ax.set_title(f"J{s.joint} R^2={s.r2:.3f}")
            ax.set_xlabel("model (A)")
            ax.set_ylabel("meas (A)")
        fig.savefig(grid_path, dpi=150)
        plt.close(fig)


def main() -> int:
    args = parse_args()

    csv_paths = args.csv if args.csv else DEFAULT_LOGS

    motor_model_path = args.motor_model or resolve_motor_model_path()
    try:
        joint_models = load_motor_model_csv(motor_model_path)
        print(f"Loaded {len(joint_models)} joint model entries from {motor_model_path}")
    except Exception as exc:
        raise SystemExit(f"Failed to load motor model: {exc}") from exc
    if len(joint_models) != len(JOINTS):
        print(
            f"Warning: motor model has {len(joint_models)} entries, expected {len(JOINTS)}."
        )

    vel_thresh = None if args.mode == "full" else args.vel_threshold_deg
    rows = load_rows(csv_paths, vel_thresh)
    if not rows:
        print("No samples after filtering. Try increasing --vel-threshold-deg.", file=sys.stderr)
        return 2

    # Build gravity model
    if args.mode == "gravity":
        joint_names = [f"joint_{j}" for j in JOINTS]
        try:
            model, backend = build_gravity_model(
                args.urdf, args.root_link, args.tip_link, joint_names, backend=args.backend
            )
            print(f"Using gravity backend: {backend}")
        except Exception as exc:
            raise SystemExit(f"Failed to build gravity model: {exc}") from exc
        compute_model(rows, model, joint_models)
    else:
        print("Using full dynamics model (Pinocchio rnea)")
        compute_full_dynamics(rows, args.urdf, joint_models)

    stats: List[FitStats] = []
    for j in JOINTS:
        xs = [row[f"current_model_{j}"] for row in rows]
        ys = [row[f"current_{j}"] for row in rows]
        slope, intercept, r2, rmse, bias, corr = fit_line_full(xs, ys)
        stats.append(
            FitStats(
                joint=j,
                n=len(xs),
                slope=slope,
                intercept=intercept,
                r2=r2,
                rmse=rmse,
                bias=bias,
                corr=corr,
            )
        )

    write_summary(stats, args.summary_csv, args.summary_json)
    print(f"Summary written to {args.summary_csv}")

    if not args.no_plots:
        plot_per_joint(rows, stats, args.plots_dir, args.plot_grid)
        print(f"Plots written to {args.plots_dir}")
        if args.plot_grid:
            print(f"Grid plot written to {args.plot_grid}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
