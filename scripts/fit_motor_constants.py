#!/usr/bin/env python3
"""
Fit per-joint affine motor current model from CSV logs.

Model (per joint): i = a * tau + b
Then: Kt = 1/a, i0 = b

Default torque source: physics gravity model from URDF.
"""

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple


JOINTS = [1, 2, 3, 4, 5, 6]
DEFAULT_URDF = "src/ros2_kortex/kortex_description/robots/gen3_lite.urdf"


@dataclass
class FitResult:
    joint: int
    n: int
    a: float
    b: float
    kt: float
    r2: float


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Fit per-joint motor constants from moveit current logs."
    )
    p.add_argument(
        "--csv",
        default="log/moveit_current_log.csv",
        help="Path to CSV log (default: log/moveit_current_log.csv)",
    )
    p.add_argument(
        "--aggregate",
        choices=["mean", "median", "none"],
        default="mean",
        help="Aggregate samples per target_idx before fitting.",
    )
    p.add_argument(
        "--velocity-threshold",
        type=float,
        default=0.02,
        help="Max abs joint velocity allowed (rad/s) for static-pose filtering.",
    )
    p.add_argument(
        "--torque-source",
        choices=["gravity", "motor_torque"],
        default="gravity",
        help="Torque source to use for tau (default: gravity).",
    )
    p.add_argument(
        "--urdf",
        default=DEFAULT_URDF,
        help=f"URDF path for gravity model (default: {DEFAULT_URDF})",
    )
    p.add_argument(
        "--out-csv",
        default="",
        help="Optional output CSV path for fit results.",
    )
    p.add_argument(
        "--out-json",
        default="",
        help="Optional output JSON path for fit results.",
    )
    p.add_argument(
        "--plot",
        action="store_true",
        help="Show fit plots (requires matplotlib).",
    )
    p.add_argument(
        "--plot-out",
        default="",
        help="Optional path to save plot image (e.g., log/fit.png).",
    )
    return p.parse_args()


def require_columns(header: Iterable[str], columns: Iterable[str], csv_path: str) -> None:
    missing = [c for c in columns if c not in header]
    if missing:
        msg = (
            f"Missing required columns in {csv_path}: {', '.join(missing)}\n"
            "Hint: log/moveit_current_log_full.csv includes motor current/torque columns."
        )
        raise SystemExit(msg)


def median(values: List[float]) -> float:
    s = sorted(values)
    n = len(s)
    if n == 0:
        return float("nan")
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


def aggregate_rows(rows: List[Dict[str, float]], aggregate: str) -> List[Dict[str, float]]:
    if aggregate == "none":
        return rows

    grouped: Dict[int, List[Dict[str, float]]] = defaultdict(list)
    for row in rows:
        grouped[int(row["target_idx"])].append(row)

    aggregated: List[Dict[str, float]] = []
    for _, samples in sorted(grouped.items(), key=lambda x: x[0]):
        out: Dict[str, float] = {"target_idx": samples[0]["target_idx"]}
        keys = samples[0].keys()
        for k in keys:
            if k == "target_idx":
                continue
            vals = [s[k] for s in samples]
            if aggregate == "mean":
                out[k] = sum(vals) / len(vals)
            else:
                out[k] = median(vals)
        aggregated.append(out)
    return aggregated


def fit_affine(pairs: List[Tuple[float, float]]) -> Tuple[float, float, float]:
    # pairs: (tau, current)
    n = len(pairs)
    if n < 2:
        return float("nan"), float("nan"), float("nan")
    xs = [p[0] for p in pairs]
    ys = [p[1] for p in pairs]
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    var_x = sum((x - mean_x) ** 2 for x in xs)
    if var_x == 0.0:
        return float("nan"), float("nan"), float("nan")
    cov_xy = sum((x - mean_x) * (y - mean_y) for x, y in pairs)
    a = cov_xy / var_x
    b = mean_y - a * mean_x

    # R^2
    ss_tot = sum((y - mean_y) ** 2 for y in ys)
    ss_res = sum((y - (a * x + b)) ** 2 for x, y in pairs)
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else float("nan")
    return a, b, r2


def try_import_pinocchio() -> "object":
    try:
        import pinocchio as pin  # type: ignore
    except Exception as exc:
        msg = (
            "Pinocchio is required for --torque-source=gravity but is not available.\n"
            "Run this inside the ROS 2 Jazzy container where pinocchio is installed, or\n"
            "switch to --torque-source=motor_torque.\n"
            f"Original import error: {exc}"
        )
        raise SystemExit(msg) from exc
    return pin


def build_gravity_model(pin: "object", urdf_path: str) -> Tuple["object", "object"]:
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    return model, data


def joint_name_for_index(j: int) -> str:
    return f"joint_{j}"


def fill_configuration(model: "object", joint_positions: Dict[int, float]) -> "object":
    import numpy as np
    q = np.zeros(model.nq)
    for j, pos in joint_positions.items():
        name = joint_name_for_index(j)
        jid = model.getJointId(name)
        if jid == 0:
            raise SystemExit(f"Joint '{name}' not found in URDF model.")
        idx = model.joints[jid].idx_q
        nq = model.joints[jid].nq
        if nq != 1:
            raise SystemExit(f"Joint '{name}' expected 1 DoF, got {nq}.")
        q[idx] = pos
    return q


def gravity_torque(
    pin: "object", model: "object", data: "object", joint_positions: Dict[int, float]
) -> Dict[int, float]:
    q = fill_configuration(model, joint_positions)
    tau = pin.computeGeneralizedGravity(model, data, q)
    out: Dict[int, float] = {}
    for j in JOINTS:
        name = joint_name_for_index(j)
        jid = model.getJointId(name)
        idx = model.joints[jid].idx_v
        out[j] = float(tau[idx])
    return out


def plot_fits(
    rows: List[Dict[str, float]],
    results: List[FitResult],
    plot_out: str,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(
            "matplotlib is required for plotting. Install it in the devcontainer or "
            "use --plot-out after installing matplotlib."
        ) from exc

    fig, axes = plt.subplots(2, 3, figsize=(12, 6), constrained_layout=True)
    axes = axes.flatten()
    for idx, r in enumerate(results):
        ax = axes[idx]
        xs = [row[f"tau_{r.joint}"] for row in rows]
        ys = [row[f"motor_current_{r.joint}"] for row in rows]
        ax.scatter(xs, ys, s=12, alpha=0.6)
        if not math.isnan(r.a):
            x_min = min(xs)
            x_max = max(xs)
            x_line = [x_min, x_max]
            y_line = [r.a * x + r.b for x in x_line]
            ax.plot(x_line, y_line, color="red", linewidth=2)
        ax.set_title(f"joint {r.joint} (R^2={r.r2:.3f})")
        ax.set_xlabel("tau (Nm)")
        ax.set_ylabel("current (A)")

    if plot_out:
        fig.savefig(plot_out, dpi=150)
        print(f"Plot saved to {plot_out}")
    else:
        plt.show()


def main() -> int:
    args = parse_args()

    with open(args.csv, "r", newline="") as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames or []

        required = ["target_idx"]
        for j in JOINTS:
            required.append(f"joint_{j}_velocity")
            required.append(f"motor_current_{j}")
            if args.torque_source == "motor_torque":
                required.append(f"motor_torque_{j}")
            else:
                required.append(f"joint_{j}_position")

        require_columns(header, required, args.csv)

        if args.torque_source == "gravity":
            pin = try_import_pinocchio()
            model, data = build_gravity_model(pin, args.urdf)

        rows: List[Dict[str, float]] = []
        for row in reader:
            try:
                max_abs_vel = max(abs(float(row[f"joint_{j}_velocity"])) for j in JOINTS)
            except ValueError:
                continue
            if max_abs_vel > args.velocity_threshold:
                continue

            parsed: Dict[str, float] = {"target_idx": float(row["target_idx"])}
            for j in JOINTS:
                parsed[f"motor_current_{j}"] = float(row[f"motor_current_{j}"])

            if args.torque_source == "motor_torque":
                for j in JOINTS:
                    parsed[f"tau_{j}"] = float(row[f"motor_torque_{j}"])
            else:
                joint_positions = {j: float(row[f"joint_{j}_position"]) for j in JOINTS}
                tau = gravity_torque(pin, model, data, joint_positions)
                for j in JOINTS:
                    parsed[f"tau_{j}"] = tau[j]

            rows.append(parsed)

    if not rows:
        print("No samples remained after filtering. Try increasing velocity threshold.", file=sys.stderr)
        return 2

    rows = aggregate_rows(rows, args.aggregate)

    results: List[FitResult] = []
    for j in JOINTS:
        pairs = [(r[f"tau_{j}"], r[f"motor_current_{j}"]) for r in rows]
        a, b, r2 = fit_affine(pairs)
        kt = 1.0 / a if a not in (0.0, float("nan")) else float("nan")
        results.append(FitResult(joint=j, n=len(pairs), a=a, b=b, kt=kt, r2=r2))

    if args.torque_source == "gravity":
        print(f"Using gravity torque from URDF: {args.urdf}")
    else:
        print("Using motor_torque_* as proxy torque (guidance only).")
    print(f"Samples used: {len(rows)} (aggregate={args.aggregate}, vel_thresh={args.velocity_threshold})")
    print("")
    print("joint  n    a (A/Nm)    b (A)    Kt (Nm/A)   R^2")
    for r in results:
        print(
            f"{r.joint:>5}  {r.n:>4}  {r.a:>10.6f}  {r.b:>7.4f}  {r.kt:>11.6f}  {r.r2:>5.3f}"
        )

    if args.out_csv:
        with open(args.out_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["joint", "n", "a", "b", "kt", "r2"])
            for r in results:
                w.writerow([r.joint, r.n, r.a, r.b, r.kt, r.r2])

    if args.out_json:
        with open(args.out_json, "w", newline="") as f:
            json.dump([r.__dict__ for r in results], f, indent=2)

    if args.plot or args.plot_out:
        plot_fits(rows, results, args.plot_out)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
