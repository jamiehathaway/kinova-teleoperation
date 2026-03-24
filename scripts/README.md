# Scripts

This folder contains analysis scripts for motor model fitting and gravity sanity checks. All scripts are intended to run inside the ROS 2 Jazzy devcontainer (not on the host).

Motor model configuration shared by C++ and Python:
- `src/kinova_teleoperation/config/motor_model.csv`
- Override path with `KINOVA_TELEOP_MOTOR_MODEL` if needed.
Columns: `actuator_idx, actuator_id, torque_constant, gear_ratio, gear_efficiency, bias, coulomb_friction, viscous_damping`

## validate_gravity_comp.py

Purpose: sanity‑check model‑based compensation by comparing **model‑predicted current** against measured actuator current from free‑drive logs.

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
- X‑axis is **model gravity‑comp current** (gravity torque mapped through the motor model).
- Y‑axis is **measured actuator current** from the log.
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
- Motor model is loaded from `motor_model.csv` and used to map torque to current (with the same clamp as C++).
- Default backend is Pinocchio for ROS 2 Jazzy. Use `--backend kdl` if `kdl_parser_py` is available.
- Use `--mode full` to compute full dynamics (gravity + coriolis + inertia) with Pinocchio and all samples.

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

## plot_motor_calibration.py

Purpose: plot **measured current** and **model gravity torque** against joint position for each motor calibration log.

Usage:
```bash
python3 src/kinova_teleoperation/scripts/plot_motor_calibration.py
```

Outputs (in a subfolder to avoid clutter):
- Per‑joint plots: `log/motor_calibration_plots/joint1.png` … `joint6.png`
- Combined grid: `log/motor_calibration_plots/combined.png`
- Fit results: `log/motor_calibration_plots/fit_results.csv` (fits current = b + a*tau + c*sign(qdot) + d*qdot)
- Fitted motor model CSV (default): `src/kinova_teleoperation/config/motor_model_fitted.csv` (override with `--model-out`)

Plot notes:
- Blue points: measured current
- Blue line: fitted current model
- Red points: model gravity torque

Notes:
- Uses the shared gravity backend from `gravity_model_backend.py`.
- Default backend is Pinocchio for ROS 2 Jazzy. Use `--backend kdl` if `kdl_parser_py` is available.
- To compare with existing models, pass `--model-in path/to/motor_model.csv` (repeatable) or `--model-in-dir src/kinova_teleoperation/config`.
- Use `--no-fit` to skip fitting and only overlay loaded models.
