#!/usr/bin/env python3
"""Plot current and model gravity torque vs joint position for calibration logs."""

import argparse
import csv
import os
from typing import Dict, List, Sequence

from gravity_model_backend import build_gravity_model
from motor_model_utils import require_columns


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
    return p.parse_args()


def load_log(path: str) -> List[Dict[str, float]]:
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames or []
        required = ["timestamp_ns", "sample_idx"]
        for j in JOINTS:
            required.append(f"actuator_{j}_position")
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
                parsed[f"current_{j}"] = float(row[f"actuator_{j}_current"])
            rows.append(parsed)
        return rows


def compute_gravity(model, rows: List[Dict[str, float]]) -> None:
    for row in rows:
        q = [row[f"pos_{j}"] * DEG2RAD for j in JOINTS]
        tau = model.compute(q)
        for j, val in zip(JOINTS, tau):
            row[f"tau_model_{j}"] = val


def plot_joint(ax, ax_r, x: Sequence[float], current: Sequence[float], torque: Sequence[float], title: str) -> None:
    ax.scatter(x, current, s=10, alpha=0.6, color="tab:blue", label="current")
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

    joint_names = [f"joint_{j}" for j in JOINTS]
    try:
        model, backend = build_gravity_model(
            args.urdf, args.root_link, args.tip_link, joint_names, backend=args.backend
        )
    except Exception as exc:
        raise SystemExit(f"Failed to build gravity model: {exc}") from exc

    os.makedirs(args.out_dir, exist_ok=True)
    grid_path = args.grid_path or os.path.join(args.out_dir, "combined.png")

    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception as exc:
        raise SystemExit(
            "matplotlib is required for plotting. Install it in the devcontainer."
        ) from exc

    # Per-joint plots
    for j, path in zip(JOINTS, log_paths):
        rows = load_log(path)
        compute_gravity(model, rows)
        x = [row[f"pos_{j}"] for row in rows]
        y_current = [row[f"current_{j}"] for row in rows]
        y_tau = [row[f"tau_model_{j}"] for row in rows]

        # Sort for nicer plots
        combined = sorted(zip(x, y_current, y_tau), key=lambda t: t[0])
        x, y_current, y_tau = zip(*combined) if combined else ([], [], [])

        fig, ax = plt.subplots(figsize=(6, 4), constrained_layout=True)
        ax_r = ax.twinx()
        plot_joint(ax, ax_r, x, y_current, y_tau, f"Joint {j}")
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
        combined = sorted(zip(x, y_current, y_tau), key=lambda t: t[0])
        x, y_current, y_tau = zip(*combined) if combined else ([], [], [])

        ax = axes[idx]
        ax_r = ax.twinx()
        plot_joint(ax, ax_r, x, y_current, y_tau, f"J{j}")

    fig.suptitle(f"Motor calibration: current vs gravity torque ({backend})", fontsize=12)
    fig.savefig(grid_path, dpi=150)
    plt.close(fig)

    print(f"Plots written to {args.out_dir}")
    print(f"Combined plot written to {grid_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
