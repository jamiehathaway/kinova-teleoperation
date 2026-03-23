#!/usr/bin/env python3
"""Plot current and model gravity torque vs joint position for calibration logs."""

import argparse
import csv
import glob
import math
import os
from typing import Dict, List, Sequence, Tuple

from gravity_model_backend import build_gravity_model
from motor_model_utils import (
    fit_drive_model,
    load_motor_model_csv,
    require_columns,
    resolve_motor_model_path,
    torque_to_current,
)


DEG2RAD = 3.141592653589793 / 180.0
JOINTS = [1, 2, 3, 4, 5, 6]
DEFAULT_URDF = "src/ros2_kortex/kortex_description/robots/gen3_lite.urdf"
DEFAULT_ROOT_LINK = "base_link"
DEFAULT_TIP_LINK = "tool_frame"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Plot current and gravity torque vs joint position for calibration logs."
    )
    p.add_argument(
        "--log-dir",
        default="log",
        help="Directory containing motor_calibration_log_j*.csv (default: log).",
    )
    p.add_argument(
        "--out-dir",
        default="log/motor_calibration_plots",
        help="Output directory for plots (default: log/motor_calibration_plots).",
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
        "--grid-path",
        default="",
        help="Optional combined grid plot path (default: out-dir/combined.png).",
    )
    p.add_argument(
        "--no-fit",
        action="store_true",
        help="Skip fitting the drive model (plots measured + loaded model only).",
    )
    p.add_argument(
        "--model-in",
        action="append",
        default=None,
        help="Motor model CSV to overlay (repeatable).",
    )
    p.add_argument(
        "--model-in-dir",
        default="",
        help="Directory to load motor_model*.csv overlays from.",
    )
    p.add_argument(
        "--motor-model",
        default="",
        help="Base motor_model.csv for actuator mapping (default: resolved).",
    )
    p.add_argument(
        "--model-out",
        default="src/kinova_teleoperation/config/motor_model_fitted.csv",
        help="Output path for fitted motor model CSV.",
    )
    return p.parse_args()


def load_log(path: str) -> List[Dict[str, float]]:
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames or []
        required = ["timestamp_ns", "sample_idx"]
        for j in JOINTS:
            required.append(f"actuator_{j}_position")
            required.append(f"actuator_{j}_velocity")
            required.append(f"actuator_{j}_current")
        try:
            require_columns(header, required, path)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc

        rows: List[Dict[str, float]] = []
        for row in reader:
            parsed: Dict[str, float] = {}
            for j in JOINTS:
                parsed[f"pos_{j}"] = float(row[f"actuator_{j}_position"])
                parsed[f"vel_{j}"] = float(row[f"actuator_{j}_velocity"]) * DEG2RAD
                parsed[f"current_{j}"] = float(row[f"actuator_{j}_current"])
            rows.append(parsed)
        return rows


def compute_gravity(model, rows: List[Dict[str, float]]) -> None:
    for row in rows:
        q = [row[f"pos_{j}"] * DEG2RAD for j in JOINTS]
        tau = model.compute(q)
        for j, val in zip(JOINTS, tau):
            row[f"tau_model_{j}"] = val


def plot_joint(
    ax,
    ax_r,
    x: Sequence[float],
    current: Sequence[float],
    torque: Sequence[float],
    fit_current: Sequence[float],
    model_lines: List[Tuple[str, Sequence[float]]],
    title: str,
) -> None:
    ax.scatter(x, current, s=10, alpha=0.6, color="tab:blue", label="current")
    if fit_current:
        ax.plot(x, fit_current, color="k", linewidth=1.5, label="fit current")
    if model_lines:
        import matplotlib.cm as cm  # type: ignore

        cmap = cm.get_cmap("tab10")
        for idx, (label, series) in enumerate(model_lines):
            # Skip tab10 blue (0) and red (3) to avoid clashing with data/torque colors.
            color_idx = 2 + idx
            color = cmap(color_idx % 10)
            ax.plot(x, series, linewidth=1.2, label=label, color=color)
    ax_r.scatter(x, torque, s=10, alpha=0.6, color="tab:red", label="gravity torque")
    ax.set_title(title)
    ax.set_xlabel("joint position (deg)")
    ax.set_ylabel("current (A)")
    ax_r.set_ylabel("gravity torque (Nm)")


def main() -> int:
    args = parse_args()

    log_paths = [
        os.path.join(args.log_dir, f"motor_calibration_log_j{j}.csv") for j in JOINTS
    ]
    for path in log_paths:
        if not os.path.exists(path):
            raise SystemExit(f"Missing log file: {path}")

    base_model_path = args.motor_model or resolve_motor_model_path()
    try:
        base_models = load_motor_model_csv(base_model_path)
    except Exception as exc:
        raise SystemExit(f"Failed to load base motor model: {exc}") from exc
    if len(base_models) != len(JOINTS):
        print(
            f"Warning: motor model has {len(base_models)} entries, expected {len(JOINTS)}."
        )

    joint_names = [f"joint_{j}" for j in JOINTS]
    try:
        model, backend = build_gravity_model(
            args.urdf, args.root_link, args.tip_link, joint_names, backend=args.backend
        )
    except Exception as exc:
        raise SystemExit(f"Failed to build gravity model: {exc}") from exc

    overlay_paths: List[str] = []
    if args.model_in:
        overlay_paths.extend(args.model_in)
    if args.model_in_dir:
        overlay_paths.extend(
            sorted(glob.glob(os.path.join(args.model_in_dir, "motor_model*.csv")))
        )

    overlay_models: List[Tuple[str, List]] = []
    for path in overlay_paths:
        try:
            overlay_models.append((os.path.basename(path), load_motor_model_csv(path)))
        except Exception as exc:
            raise SystemExit(f"Failed to load overlay model {path}: {exc}") from exc

    os.makedirs(args.out_dir, exist_ok=True)
    grid_path = args.grid_path or os.path.join(args.out_dir, "combined.png")
    vel_path = os.path.join(os.path.dirname(grid_path), "combined_vel.png")

    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception as exc:
        raise SystemExit(
            "matplotlib is required for plotting. Install it in the devcontainer."
        ) from exc

    # Per-joint plots + fit summary
    fit_rows: List[Dict[str, float]] = []
    for j, path in zip(JOINTS, log_paths):
        rows = load_log(path)
        compute_gravity(model, rows)
        x = [row[f"pos_{j}"] for row in rows]
        y_current = [row[f"current_{j}"] for row in rows]
        y_tau = [row[f"tau_model_{j}"] for row in rows]
        y_vel = [row[f"vel_{j}"] for row in rows]

        fit_current: List[float] = []
        if not args.no_fit:
            a, b, c, d, r2, rmse = fit_drive_model(y_tau, y_current, y_vel)
            if a == 0.0 or math.isnan(a):
                kt = float("nan")
                fc = float("nan")
                bv = float("nan")
            else:
                kt = 1.0 / a
                fc = c / a
                bv = d / a
            fit_rows.append(
                {
                    "joint": j,
                    "a": a,
                    "b": b,
                    "c": c,
                    "d": d,
                    "kt": kt,
                    "fc": fc,
                    "bv": bv,
                    "r2": r2,
                    "rmse": rmse,
                }
            )
            for tau, vel in zip(y_tau, y_vel):
                fit_current.append(
                    b
                    + a * tau
                    + c * (1.0 if vel > 0 else -1.0 if vel < 0 else 0.0)
                    + d * vel
                )

        model_lines: List[Tuple[str, Sequence[float]]] = []
        for label, models in overlay_models:
            by_idx = {m.actuator_idx: m for m in models}
            model_entry = by_idx.get(j - 1)
            if not model_entry:
                continue
            series = [
                torque_to_current(tau, vel, model_entry, clamp=None)
                for tau, vel in zip(y_tau, y_vel)
            ]
            model_lines.append((label, series))

        fig, ax = plt.subplots(figsize=(6, 4), constrained_layout=True)
        ax_r = ax.twinx()
        plot_joint(ax, ax_r, x, y_current, y_tau, fit_current, model_lines, f"Joint {j}")
        fig.savefig(os.path.join(args.out_dir, f"joint{j}.png"), dpi=150)
        plt.close(fig)

    # Combined grid plot
    fig, axes = plt.subplots(2, 3, figsize=(12, 6), constrained_layout=True)
    axes = axes.flatten()
    for idx, (j, path) in enumerate(zip(JOINTS, log_paths)):
        rows = load_log(path)
        compute_gravity(model, rows)
        x = [row[f"pos_{j}"] for row in rows]
        y_current = [row[f"current_{j}"] for row in rows]
        y_tau = [row[f"tau_model_{j}"] for row in rows]
        y_vel = [row[f"vel_{j}"] for row in rows]

        fit_current: List[float] = []
        if not args.no_fit:
            fit_row = next(r for r in fit_rows if r["joint"] == j)
            a, b, c, d = fit_row["a"], fit_row["b"], fit_row["c"], fit_row["d"]
            for tau, vel in zip(y_tau, y_vel):
                fit_current.append(
                    b
                    + a * tau
                    + c * (1.0 if vel > 0 else -1.0 if vel < 0 else 0.0)
                    + d * vel
                )

        model_lines: List[Tuple[str, Sequence[float]]] = []
        for label, models in overlay_models:
            by_idx = {m.actuator_idx: m for m in models}
            model_entry = by_idx.get(j - 1)
            if not model_entry:
                continue
            series = [
                torque_to_current(tau, vel, model_entry, clamp=None)
                for tau, vel in zip(y_tau, y_vel)
            ]
            model_lines.append((label, series))

        ax = axes[idx]
        ax_r = ax.twinx()
        plot_joint(ax, ax_r, x, y_current, y_tau, fit_current, model_lines, f"J{j}")

    fig.suptitle(f"Motor calibration: current vs gravity torque ({backend})", fontsize=12)
    fig.savefig(grid_path, dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 3, figsize=(12, 6), constrained_layout=True)
    axes = axes.flatten()
    for idx, (j, path) in enumerate(zip(JOINTS, log_paths)):
        rows = load_log(path)
        x = [row[f"pos_{j}"] for row in rows]
        y_vel = [row[f"vel_{j}"] for row in rows]
        ax = axes[idx]
        ax.scatter(x, y_vel, s=10, alpha=0.6, color="tab:blue", label="current")
        ax.set_title(f"J{j}")
        ax.set_xlabel("joint position (deg)")
        ax.set_ylabel("velocity (rad/s)")
    fig.suptitle(f"Motor calibration: position vs. velocity ({backend})", fontsize=12)
    fig.savefig(vel_path, dpi=150)
    plt.close(fig)

    print(f"Plots written to {args.out_dir}")
    print(f"Combined plot written to {grid_path}")

    if not args.no_fit:
        fit_path = os.path.join(args.out_dir, "fit_results.csv")
        with open(fit_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "joint",
                    "a",
                    "b",
                    "c",
                    "d",
                    "kt",
                    "coulomb_friction",
                    "viscous_damping",
                    "r2",
                    "rmse",
                ]
            )
            for row in fit_rows:
                writer.writerow(
                    [
                        row["joint"],
                        row["a"],
                        row["b"],
                        row["c"],
                        row["d"],
                        row["kt"],
                        row["fc"],
                        row["bv"],
                        row["r2"],
                        row["rmse"],
                    ]
                )

        print(f"Fit results written to {fit_path}")

        if args.model_out:
            model_out_dir = os.path.dirname(args.model_out) or "."
            os.makedirs(model_out_dir, exist_ok=True)
            by_joint = {row["joint"]: row for row in fit_rows}
            base_by_idx = {m.actuator_idx: m for m in base_models}
            with open(args.model_out, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(
                    [
                        "actuator_idx",
                        "actuator_id",
                        "torque_constant",
                        "gear_ratio",
                        "gear_efficiency",
                        "bias",
                        "coulomb_friction",
                        "viscous_damping",
                    ]
                )
                for j in JOINTS:
                    row = by_joint[j]
                    base = base_by_idx.get(j - 1)
                    actuator_idx = base.actuator_idx if base else (j - 1)
                    actuator_id = base.actuator_id if base else j
                    w.writerow(
                        [
                            actuator_idx,
                            actuator_id,
                            row["kt"],
                            1.0,
                            1.0,
                            row["b"],
                            row["fc"],
                            row["bv"],
                        ]
                    )
            print(f"Fitted motor model written to {args.model_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
