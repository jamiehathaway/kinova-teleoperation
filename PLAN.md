# Plan: Adapt Kinova Leader-Follower Example to ROS 2 Jazzy with Force Feedback

## 1) What the current repo does (baseline)

### 1.1 Runtime flow
1. Connect to two Kinova robots using Kortex API (`Robot` class in `api_setup.cpp`).
2. Move both to `Home` in `SINGLE_LEVEL_SERVOING` (`move_to_home_position`).
3. Wait for wrist button press on leader (`status_flags` bits 26/27 in `read_buttons_blocking`).
4. Run teleop loop (`lead`) for a fixed duration.

### 1.2 Control mode used per robot today
- Leader:
  - Base servoing mode: `LOW_LEVEL_SERVOING`.
  - Actuators switched to `ActuatorConfig::ControlMode::CURRENT` for selected joints (currently all 6 logical joints).
  - In cyclic loop, joint position command is held at measured position to avoid following error while in current mode.
- Follower:
  - Base servoing mode: `LOW_LEVEL_SERVOING`.
  - Follower tracks leader **joint positions** (with initial offset compensation), i.e. effectively position-following at low-level cyclic interface.
  - No explicit switch to current mode in follower.

### 1.3 Capabilities and limitations of current implementation
- Capabilities:
  - Real-time cyclic command/feedback using Kortex UDP channel.
  - Access to actuator feedback including position and motor current (`current_motor` in `BaseCyclic`).
  - Access to interconnect button status on leader wrist.
- Limitations vs target:
  - No ROS 2 integration.
  - No Cartesian follower interface.
  - No force/torque feedback ingestion and no haptic rendering loop.
  - No explicit wrench-to-joint-torque mapping.

## 2) Target behavior and architecture (ROS 2 Jazzy)

### 2.1 Desired system behavior
- Leader side (Kinova Gen3 Lite + Kortex):
  - Read leader state in real time.
  - Publish follower Cartesian pose commands over ROS topics.
  - Subscribe to follower force/torque (`geometry_msgs/msg/WrenchStamped`).
  - Compute leader joint torques from wrench and command equivalent **joint currents** to render haptic force.
- Follower side (external ROS-only robot):
  - Accepts only Cartesian pose targets via topic.
  - Publishes F/T sensor topic.

### 2.2 Proposed ROS 2 nodes
1. `leader_kortex_driver_node` (C++):
   - Owns Kortex session, cyclic control loop (1 kHz target).
   - Publishes leader state (joint + EE pose) for debugging.
   - Applies current commands to leader joints.
2. `teleop_bridge_node` (C++):
   - Converts leader EE motion to follower pose command topic.
   - Handles clutch/button gating, frame transforms, scaling.
3. `haptic_render_node` (C++):
   - Subscribes follower wrench.
   - Filters/scales/transforms wrench.
   - Computes leader joint torque command: `tau = J(q)^T * wrench`.
   - Converts torque to current command using identified/estimated gains.

Note: For lowest latency, (1) and (3) can be merged into one real-time process if needed.

### 2.3 ROS 2 topic contract (proposed)
- Publish:
  - `/teleop/follower/pose_cmd` : `geometry_msgs/msg/PoseStamped`
  - `/leader/joint_states` : `sensor_msgs/msg/JointState`
  - `/leader/ee_pose` : `geometry_msgs/msg/PoseStamped`
  - `/leader/haptics/debug` : custom or `std_msgs/msg/Float64MultiArray`
- Subscribe:
  - `/follower/ft` : `geometry_msgs/msg/WrenchStamped`
  - `/leader/clutch` (optional override) : `std_msgs/msg/Bool`

Use `tf2` for frame-consistent wrench transformation.

## 3) Core control math to implement

### 3.1 Wrench reflection from follower to leader
Let follower wrench in follower tool frame be:

\[
\mathbf{w}_f = \begin{bmatrix} \mathbf{F}_f \\ \boldsymbol{\tau}_f \end{bmatrix}
\]

Transform to leader tool frame using rigid transform (rotation \(\mathbf{R}\), position offset \(\mathbf{p}\)):

\[
\mathbf{F}_\ell = \mathbf{R}\,\mathbf{F}_f
\]
\[
\boldsymbol{\tau}_\ell = \mathbf{R}\,\boldsymbol{\tau}_f + \mathbf{p} \times \mathbf{F}_\ell
\]

Apply reflection and gain:

\[
\mathbf{w}_{render} = -\mathbf{G}\,\mathbf{w}_\ell
\]

where \(\mathbf{G}=\mathrm{diag}(g_x,g_y,g_z,g_{rx},g_{ry},g_{rz})\).

### 3.2 Filtering and deadband
Use first-order low-pass filter per wrench component:

\[
\hat{w}_k = \alpha w_k + (1-\alpha)\hat{w}_{k-1},\quad \alpha = \frac{2\pi f_c\,\Delta t}{1+2\pi f_c\,\Delta t}
\]

Apply deadband \(d_i\): if \(|\hat{w}_i|<d_i\), set to 0.

### 3.3 Wrench to leader joint torques
Compute leader geometric Jacobian at current joint state \(\mathbf{J}(\mathbf{q})\in\mathbb{R}^{6\times n}\):

\[
\boldsymbol{\tau}_{cmd} = \mathbf{J}(\mathbf{q})^T\,\mathbf{w}_{render}
\]

Add joint damping to stabilize free-space behavior:

\[
\boldsymbol{\tau}_{cmd} \leftarrow \boldsymbol{\tau}_{cmd} - \mathbf{B}\,\dot{\mathbf{q}}
\]

Saturate per-joint torque command: \(\tau_{min,j} \le \tau_j \le \tau_{max,j}\).

### 3.4 Torque to current for Gen3 Lite (no torque sensors)
You can command motor current but motor constants are unknown.

Primary model:

\[
\tau_j \approx K_{t,j}(i_j - i_{0,j}) - F_{c,j}\,\mathrm{sgn}(\dot{q}_j) - B_{v,j}\dot{q}_j
\]

Commanded current (using estimated parameters):

\[
i_{cmd,j} = i_{0,j} + \frac{\tau_{cmd,j} + F_{c,j}\,\mathrm{sgn}(\dot{q}_j) + B_{v,j}\dot{q}_j}{\hat{K}_{t,j}}
\]

If \(\hat{K}_{t,j}\) is unknown initially, start with conservative scalar gains:

\[
i_{cmd,j} = i_{bias,j} + \alpha_j\,\tau_{cmd,j}
\]

and tune \(\alpha_j\) with strict saturation and watchdog limits.

## 4) Practical solution for unknown motor constants

Because Gen3 Lite lacks direct torque sensing, use a staged identification strategy.

### 4.1 Stage A: Safe initial deployment (no hard ID dependency)
- Use small current limits and low wrench gains.
- Use proportional torque-to-current scaling `alpha_j` per joint.
- Validate subjective haptic direction and relative magnitude first.

### 4.2 Stage B: Offline parameter identification (recommended)
1. Build/load a dynamics model for leader arm (URDF + inertial params; use Pinocchio or KDL).
2. Collect logs at multiple static poses with known payloads.
   - Logged: `q`, `qdot`, `current_motor`.
   - Known: gravity torque from model at each pose/payload.
3. Fit per-joint affine map using least squares:

\[
i_j = a_j\,\tau_{model,j} + b_j
\]

Then:

\[
\hat{K}_{t,j} = 1/a_j,\quad i_{0,j}=b_j
\]

4. Add dynamic trajectories to estimate friction terms \(F_c, B_v\).

### 4.3 Stage C: Online adaptation (optional)
- Slowly adapt `alpha_j` based on consistency between expected and observed behavior.
- Keep adaptation bounded; disable adaptation on faults/saturation.

Important: treat `ActuatorFeedback::torque()` (if available) as a diagnostic estimate, not a high-confidence torque sensor.

## 5) Detailed implementation plan (step-by-step)

### Step 1: Create ROS 2 package skeleton (no install in this step)
- New package: `kinova_teleop_ros2` (ament_cmake, C++17).
- Dependencies:
  - `rclcpp`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `Eigen3`.
  - Kortex API linked similarly to current CMake setup.
- Add launch file placeholder for all nodes.

### Step 2: Port Kortex connection class into ROS 2-friendly component
- Refactor current `Robot` class into RAII-safe class used by node(s).
- Add explicit lifecycle methods: `connect()`, `start_low_level()`, `stop_low_level()`, `cleanup()`.
- Preserve current session setup and fault clearing logic.

### Step 3: Implement leader cyclic control loop
- Dedicated real-time thread (1 kHz target) in `leader_kortex_driver_node`.
- Read cyclic feedback each cycle.
- Maintain command frame IDs and command IDs as in current `lead()`.
- Publish leader `JointState` and EE pose at lower ROS rate (e.g., 100 Hz) to avoid DDS overload.

### Step 4: Implement clutch/button gating
- Reuse current bit-mask logic for wrist buttons.
- Only allow follower pose publish + haptic render when clutch is pressed.
- On clutch release: ramp currents to bias/zero smoothly (no step changes).

### Step 5: Implement leader EE kinematics and follower pose command
- Compute leader EE pose from joint state.
- Define calibration transform from leader EE frame to follower command frame.
- Publish `PoseStamped` to `/teleop/follower/pose_cmd`.
- Add pose scaling and workspace clamps.

### Step 6: Implement follower wrench subscriber and preprocessing
- Subscribe `/follower/ft` (`WrenchStamped`).
- Validate timestamp freshness (drop stale samples).
- Transform wrench to leader tool frame (`tf2`).
- Apply low-pass filter, deadband, saturation.

### Step 7: Implement haptic torque computation and current command
- Compute Jacobian `J(q)` of leader.
- Compute `tau_cmd = J^T w_render - B*qdot`.
- Convert to current via identified or tuned gains.
- Write currents into cyclic command message (`set_current_motor`).
- Keep position command equal to measured position for following-error protection (existing pattern).

### Step 8: Add safety supervisor
- Watchdogs:
  - comms timeout leader/follower topics,
  - stale wrench timeout,
  - current saturation duration,
  - button released/fault detected.
- Emergency behavior:
  - zero/ramp-down currents,
  - restore `POSITION` mode,
  - return to `SINGLE_LEVEL_SERVOING`.

### Step 9: Parameterization and configuration files
- YAML configs:
  - topic names, frames, filter cutoffs, gains, deadbands,
  - per-joint current limits,
  - torque-to-current parameters (`alpha` or `Kt/bias/friction`),
  - clutch behavior and ramp rates.
- Keep robot-specific settings separate from algorithm gains.

### Step 10: Validation sequence
1. Bench test with no user contact, clutch disabled.
2. Clutch + pose publishing only (no haptics).
3. Inject synthetic wrench topic and verify torque/current direction signs.
4. Very low-gain haptics with human operator.
5. Gradually increase gains while monitoring saturations and oscillation.

## 6) Acceptance criteria
- Leader publishes follower Cartesian pose commands continuously while clutch pressed.
- Follower wrench topic is consumed and transformed correctly.
- Leader receives nonzero wrench and generates corresponding current commands without instability.
- On clutch release, stale data, or fault, system safely exits haptic mode.
- All limits (wrench, torque, current, rate) are enforced and logged.

## 7) Recommended code organization
- `src/leader_kortex_driver_node.cpp`
- `src/teleop_bridge_node.cpp`
- `src/haptic_render_node.cpp`
- `src/kinematics.cpp` / `include/kinematics.hpp`
- `src/safety_supervisor.cpp` / `include/safety_supervisor.hpp`
- `config/teleop.yaml`, `config/haptics.yaml`, `config/robot_limits.yaml`
- `launch/teleop_system.launch.py`

## 8) Notes for implementer (libraries and theory shortcuts)
- Use `Eigen` for Jacobian/wrench math.
- Use `tf2` for frame-safe wrench transforms.
- For Jacobian and dynamics:
  - Preferred: Pinocchio (clean API for `J(q)` and gravity torque).
  - Alternative: Orocos KDL (lighter, common in ROS).
- Start with admittance-like conservative gains; only move to aggressive reflection after identification.

## 9) Kortex API references to consult during implementation
- Existing local API headers in `kortex_api/include/messages/BaseCyclic.pb.h` for:
  - `ActuatorCommand::set_current_motor(...)`
  - `ActuatorFeedback::current_motor()`
  - diagnostic torque and external wrench fields
- Official Kortex C++ API index (provided):
  - https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_cpp/doc/markdown/index.md

