#!/usr/bin/env python3
"""Plot current and model gravity torque vs joint position for calibration logs."""

import argparse
import csv
import glob
import math
import os
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from gravity_model_backend import build_gravity_model, GravityModelBase
from motor_model_utils import (
    fit_drive_model,
    load_motor_model_csv,
    require_columns,
    resolve_motor_model_path,
    torque_to_current,
    JointModel
)


DEG2RAD = 3.141592653589793 / 180.0
JOINTS = [1, 2, 3, 4, 5, 6] # TODO: Get rid of this hack
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
        "--plot",
        action="store_true",
        help="Generate plots for the joint data and models (if fitted or provided).",
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
        has_current_model = all(
            f"actuator_{j}_current_model" in header for j in JOINTS
        )
        has_torque_model = all(
            f"actuator_{j}_torque_model" in header for j in JOINTS
        )
        try:
            require_columns(header, required, path)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc

        rows: List[Dict[str, float]] = []
        for row in reader:
            parsed: Dict[str, float] = {}
            parsed["time_s"] = float(row[f"timestamp_ns"]) * 1e-9
            for j in JOINTS:
                parsed[f"pos_{j}"] = float(row[f"actuator_{j}_position"]) * DEG2RAD
                parsed[f"vel_{j}"] = float(row[f"actuator_{j}_velocity"]) * DEG2RAD
                parsed[f"current_{j}"] = float(row[f"actuator_{j}_current"])
                if has_current_model:
                    parsed[f"current_model_{j}"] = float(row[f"actuator_{j}_current_model"])
                if has_torque_model:
                    parsed[f"tau_logged_{j}"] = float(row[f"actuator_{j}_torque_model"])
            rows.append(parsed)
        return rows


@dataclass
class JointData():
    joint_idx: int
    x: List[float]
    y_current: List[float]
    y_tau: List[float]
    y_vel: List[float]
    y_tau_logged: List[float]


def compute_gravity(rows: List[Dict[str, float]], robot_model: GravityModelBase):
    t = np.array([row["time_s"] for row in rows])
    q = np.array([[row[f"pos_{j}"] for j in JOINTS] for row in rows])
    v = np.array([[row[f"vel_{j}"] for j in JOINTS] for row in rows])
    a = np.gradient(v, t, axis=0, edge_order=2)
    
    tau = [robot_model.compute(q[i], v[i], a[i]) for i in range(len(rows))]
    return tau


def load_joint_data(log_path: str, robot_model: GravityModelBase, joint_idx: int) -> JointData:
    rows = load_log(log_path)
    tau = compute_gravity(rows, robot_model)
    j = joint_idx
    return JointData(
        j,
        x=[row[f"pos_{j}"] for row in rows],
        y_current= [row[f"current_{j}"] for row in rows],
        y_tau=[t[j-1] for t in tau],
        y_vel=[row[f"vel_{j}"] for row in rows],
        y_tau_logged=(
            [row[f"tau_logged_{j}"] for row in rows]
            if f"tau_logged_{j}" in rows[0]
            else []
        )
    )


def plot_joint(
    ax,
    ax_r,
    joint_data: JointData,
    model_predictions: List[Tuple[str, Sequence[float]]],
    title: str,
) -> None:
    x, current, torque, torque_logged = (joint_data.x, 
                                         joint_data.y_current,
                                         joint_data.y_tau,
                                         joint_data.y_tau_logged)
    
    ax.scatter(x, current, s=10, alpha=0.6, color="tab:blue", label="current")
    if model_predictions:
        cmap = cm.get_cmap("tab10")
        for idx, (label, series) in enumerate(model_predictions):
            # Skip tab10 blue (0) and red (3) to avoid clashing with data/torque colors.
            color_idx = 2 + idx
            color = cmap(color_idx % 10)
            ax.plot(x, series, linewidth=1.2, label=label, color=color)
    ax_r.scatter(x, torque, s=10, alpha=0.6, color="tab:red", label="gravity torque (py)")
    if torque_logged:
        ax_r.scatter(
            x,
            torque_logged,
            s=10,
            alpha=0.6,
            color="tab:orange",
            label="gravity torque (logged)",
        )
    ax.set_title(title)
    ax.set_xlabel("joint position (deg)")
    ax.set_ylabel("current (A)")
    ax_r.set_ylabel("gravity torque (Nm)")


def fit_joints(joints_data: Sequence[JointData]):
    fitted_rows: List[Dict[str, float]] = []
    
    for joint_data in joints_data:
        a, b, c, d, r2, rmse = fit_drive_model(joint_data.y_tau, joint_data.y_current, joint_data.y_vel)
        if a == 0.0 or math.isnan(a):
            kt = float("nan")
            fc = float("nan")
            bv = float("nan")
        else:
            kt = 1.0 / a
            fc = c / a
            bv = d / a
        fitted_rows.append({
            "joint": joint_data.joint_idx,
            "a": a,
            "b": b,
            "c": c,
            "d": d,
            "kt": kt,
            "fc": fc,
            "bv": bv,
            "r2": r2,
            "rmse": rmse,
        })
    return fitted_rows


def get_motor_model_predictions(joint_data: JointData,
                                motor_models: List[Tuple[str, List[JointModel]]]) -> List[Tuple[str, Sequence[float]]]:
    model_predictions: List[Tuple[str, Sequence[float]]] = []
    for label, models in motor_models:
        by_idx = {m.actuator_idx: m for m in models}
        model_entry = by_idx.get(joint_data.joint_idx - 1)
        if not model_entry:
            continue
        series = [
            torque_to_current(tau, vel, model_entry, clamp=None)
            for tau, vel in zip(joint_data.y_tau, joint_data.y_vel)
        ]
        model_predictions.append((label, series))
    return model_predictions


# Per-joint plots + fit summary
def plot_joints(joints_data: Sequence[JointData], out_dir: str, motor_models: List[Tuple[str, List[JointModel]]] = []):
    for joint_data in joints_data:
        fig, ax = plt.subplots(figsize=(6, 4), constrained_layout=True)
        model_predictions = get_motor_model_predictions(joint_data, motor_models)
        ax_r = ax.twinx()
        plot_joint(
            ax, ax_r,
            joint_data,
            model_predictions,
            f"Joint {joint_data.joint_idx}"
        )
        fig.legend()
        fig.savefig(os.path.join(out_dir, f"joint{joint_data.joint_idx}.png"), dpi=150)
        plt.close(fig)


# Combined grid plot
def plot_combined_joints(joints_data: Sequence[JointData], out_dir: str, motor_models: List[Tuple[str, List[JointModel]]] = []):
    grid_path = os.path.join(out_dir, "combined.png")
    fig, axs = plt.subplots(2, 3, figsize=(12, 6), constrained_layout=True)
    axs = axs.flatten()

    for ax, joint_data in zip(axs, joints_data):
        model_predictions = get_motor_model_predictions(joint_data, motor_models)
        ax_r = ax.twinx()
        plot_joint(
            ax, ax_r,
            joint_data,
            model_predictions,
            f"J{joint_data.joint_idx}"
        )
        
    axs[0].legend()
    fig.suptitle(f"Motor calibration: current vs gravity torque", fontsize=12)
    fig.savefig(grid_path, dpi=150)
    plt.close(fig)


def plot_pos_vs_vel(joints_data: Sequence[JointData], out_dir: str):
    vel_path = os.path.join(out_dir, "combined_vel.png")
    fig, axs = plt.subplots(2, 3, figsize=(12, 6), constrained_layout=True)
    axs = axs.flatten()
    for ax, joint_data in zip(axs, joints_data):
        ax.scatter(joint_data.x, joint_data.y_vel, s=10, alpha=0.6, color="tab:blue", label="current")
        ax.set_title(f"J{joint_data.joint_idx}")
        ax.set_xlabel("joint position (deg)")
        ax.set_ylabel("velocity (rad/s)")
    fig.suptitle(f"Motor calibration: position vs. velocity", fontsize=12)
    fig.savefig(vel_path, dpi=150)
    plt.close(fig)


def output_fit_results(fitted_rows: List[Dict[str, float]], out_dir: str):
    fit_path = os.path.join(out_dir, "fit_results.csv")
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
        for row in fitted_rows:
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
    return fit_path


def fitted_rows_to_motor_model(
        fitted_rows: List[Dict[str, float]],
        motor_model_base: List[JointModel]):
    by_joint = {row["joint"]: row for row in fitted_rows}
    base_by_idx = {m.actuator_idx: m for m in motor_model_base}

    fitted_model: List[JointModel] = []
    for j in JOINTS:
        row = by_joint[j]
        base = base_by_idx.get(j - 1)
        actuator_idx = base.actuator_idx if base else (j - 1)
        actuator_id = base.actuator_id if base else j

        fitted_model.append(
            JointModel(
                actuator_idx=actuator_idx,
                actuator_id=actuator_id,
                torque_constant=row["kt"],
                gear_ratio=1.0,
                gear_efficiency=1.0,
                bias=row["b"],
                coulomb_friction=row["fc"],
                viscous_damping=row["bv"],
            )
        )
        
    return fitted_model


def output_model(motor_model: List[JointModel], out_path: str):
    model_out_dir = os.path.dirname(out_path) or "."
    os.makedirs(model_out_dir, exist_ok=True)

    with open(out_path, "w", newline="") as f:
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
        for motor in motor_model:
            w.writerow(
                [
                    motor.actuator_idx,
                    motor.actuator_id,
                    motor.torque_constant,
                    motor.gear_ratio,
                    motor.gear_efficiency,
                    motor.bias,
                    motor.coulomb_friction,
                    motor.viscous_damping,
                ]
            )

def build_robot_model(args: argparse.Namespace) -> GravityModelBase:
    joint_names = [f"joint_{j}" for j in JOINTS]
    try:
        robot_model, backend = build_gravity_model(
            args.urdf, args.root_link, args.tip_link, joint_names, backend=args.backend
        )
    except Exception as exc:
        raise SystemExit(f"Failed to build robot model: {exc}") from exc
    
    return robot_model


def build_motor_models(args: argparse.Namespace) -> List[Tuple[str, List[JointModel]]]:
    overlay_paths: List[str] = []
    if args.model_in:
        overlay_paths.extend(args.model_in)
    if args.model_in_dir:
        overlay_paths.extend(
            sorted(glob.glob(os.path.join(args.model_in_dir, "motor_model*.csv")))
        )

    overlay_models: List[Tuple[str, List[JointModel]]] = []
    for path in overlay_paths:
        try:
            motor_model = load_motor_model_csv(path)
            if len(motor_model) != len(JOINTS):
                print(
                    f"Warning: motor model has {len(motor_model)} entries, expected {len(JOINTS)}."
                )
            overlay_models.append((os.path.basename(path), motor_model))
            
        except Exception as exc:
            raise SystemExit(f"Failed to load overlay model {path}: {exc}") from exc
    
    return overlay_models


def main() -> int:
    args = parse_args()

    # "Base model", only used to assign correct joint indices to saved models
    # TODO: Remove, only here because Kinova's API indexes joints slightly differently.
    base_model_path = args.motor_model or resolve_motor_model_path()
    try:
        motor_model_base = load_motor_model_csv(base_model_path)
    except Exception as exc:
        raise SystemExit(f"Failed to load base motor model: {exc}") from exc
    if len(motor_model_base) != len(JOINTS):
        print(
            f"Warning: motor model has {len(motor_model_base)} entries, expected {len(JOINTS)}."
        )

    robot_model = build_robot_model(args)
    motor_models = build_motor_models(args)

    joints_data: List[JointData] = []
    for j in JOINTS:
        log_path = os.path.join(args.log_dir, f"motor_calibration_log_j{j}.csv")
        if not os.path.exists(log_path):
            raise SystemExit(f"Missing log file: {log_path}")
        
        joints_data.append(load_joint_data(log_path, robot_model, j))


    os.makedirs(args.out_dir, exist_ok=True)

    if not args.no_fit:
        fitted_rows = fit_joints(joints_data)
        fit_path = output_fit_results(fitted_rows, args.out_dir)
        print(f"Fit results written to {fit_path}")

        fitted_model = fitted_rows_to_motor_model(fitted_rows, motor_model_base)
        if args.model_out:
            output_model(fitted_model, args.model_out)
            print(f"Fitted motor model written to {args.model_out}")
        
        motor_models.append(("fit current", fitted_model))

    if args.plot:
        plot_joints(joints_data, args.out_dir, motor_models)
        plot_combined_joints(joints_data, args.out_dir, motor_models)
        plot_pos_vs_vel(joints_data, args.out_dir)
        print(f"Plots written to {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
