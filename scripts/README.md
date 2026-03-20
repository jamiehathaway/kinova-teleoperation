# Scripts

This folder contains analysis scripts for motor model fitting and gravity sanity checks. All scripts are intended to run inside the ROS 2 Jazzy devcontainer (not on the host).

Motor model configuration shared by C++ and Python:
- `src/kinova_teleoperation/config/motor_model.csv`
- Override path with `KINOVA_TELEOP_MOTOR_MODEL` if needed.

## validate_gravity_comp.py

Purpose: sanity‑check the gravity model used in `torque_control_test.cpp` by comparing model gravity torque against measured actuator torque from free‑drive logs.

Usage:
```bash
python3 src/kinova_teleoperation/scripts/validate_gravity_comp.py
```

Key outputs:
- Per‑joint plots: `log/gravity_sanity_joint1.png` … `joint6.png`
- Grid plot: `log/gravity_sanity_grid.png`
- Summary metrics: `log/gravity_sanity_summary.csv`

How to read the plots:
- Each point is a low‑velocity sample.
- X‑axis is **model gravity torque** (URDF/KDL or Pinocchio fallback).
- Y‑axis is **measured actuator torque** from the log.
- **1:1 dashed line** means perfect agreement (y = x).
- **Solid fit line** is the best‑fit linear relationship between model and measurement.

Interpretation cues:
- Fit line near the 1:1 line with high R² means the gravity model matches the measured torque well.
- Slope < 1 means the model predicts torque larger than measured.
- Slope > 1 means the model predicts torque smaller than measured.
- Non‑zero intercept suggests constant bias (sensor offset, unmodeled load, etc.).
- Wide scatter indicates noise, friction, or residual motion in the data.

Notes:
- The script filters to low‑velocity samples (default 1.0 deg/s). Adjust with `--vel-threshold-deg`.
- Default backend is Pinocchio for ROS 2 Jazzy. Use `--backend kdl` if `kdl_parser_py` is available.

## fit_motor_constants.py

Purpose: fit a per‑joint affine motor model from static‑pose logs.

Model:
- `current = a * torque + b`
- `Kt = 1 / a` and `i0 = b`

Usage:
```bash
python3 src/kinova_teleoperation/scripts/fit_motor_constants.py --csv log/moveit_current_log.csv
```

Plot meaning (when `--plot` or `--plot-out` is used):
- Each point is one (torque, current) sample used in the fit.
- X‑axis is **torque** (from the selected torque source).
- Y‑axis is **motor current**.
- **Solid fit line** is the least‑squares fit for `current = a * torque + b`.
- R² in the title indicates how well the linear model explains the data.

Notes:
- Default torque source is URDF gravity (`--torque-source=gravity`).
- `--torque-source=motor_torque` uses the logged torque as a proxy (guidance only).
- The script filters to low‑velocity samples and can aggregate by target index.
- Default backend is Pinocchio for ROS 2 Jazzy. Use `--backend kdl` if `kdl_parser_py` is available.
