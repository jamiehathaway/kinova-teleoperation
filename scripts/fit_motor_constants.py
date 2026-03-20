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
from typing import Dict, List

from gravity_model_backend import build_gravity_model
from motor_model_utils import fit_line, require_columns


JOINTS = [1, 2, 3, 4, 5, 6]
DEFAULT_URDF = "src/ros2_kortex/kortex_description/robots/gen3_lite.urdf"
DEFAULT_ROOT_LINK = "base_link"
DEFAULT_TIP_LINK = "tool_frame"


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


def require_columns_or_exit(header, columns, csv_path: str) -> None:
    try:
        require_columns(header, columns, csv_path)
    except ValueError as exc:
        msg = (
            f"{exc}\n"
            "Hint: log/moveit_current_log_full.csv includes motor current/torque columns."
        )
        raise SystemExit(msg) from exc


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

        require_columns_or_exit(header, required, args.csv)

        gravity_backend = ""
        if args.torque_source == "gravity":
            joint_names = [f"joint_{j}" for j in JOINTS]
            try:
                model, gravity_backend = build_gravity_model(
                    args.urdf, args.root_link, args.tip_link, joint_names
                )
            except Exception as exc:
                raise SystemExit(f"Failed to build gravity model: {exc}") from exc

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
                q = [float(row[f"joint_{j}_position"]) for j in JOINTS]
                tau = model.compute(q)
                for j, val in zip(JOINTS, tau):
                    parsed[f"tau_{j}"] = val

            rows.append(parsed)

    if not rows:
        print("No samples remained after filtering. Try increasing velocity threshold.", file=sys.stderr)
        return 2

    rows = aggregate_rows(rows, args.aggregate)

    results: List[FitResult] = []
    for j in JOINTS:
        pairs = [(r[f"tau_{j}"], r[f"motor_current_{j}"]) for r in rows]
        a, b, r2 = fit_line([p[0] for p in pairs], [p[1] for p in pairs])
        kt = 1.0 / a if a not in (0.0, float("nan")) else float("nan")
        results.append(FitResult(joint=j, n=len(pairs), a=a, b=b, kt=kt, r2=r2))

    if args.torque_source == "gravity":
        print(f"Using gravity backend: {gravity_backend}")
        print(f"URDF: {args.urdf}")
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
