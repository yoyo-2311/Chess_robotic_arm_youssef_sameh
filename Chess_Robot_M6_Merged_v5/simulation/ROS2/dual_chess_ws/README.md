# Dual Chess Robot — ROS 2 Workspace (v1.3.3.4 + audit merge + control stage 1)

ROS 2 Jazzy · Ubuntu 24.04 · Gazebo Sim 8 (Harmonic) through `ros_gz`.
This workspace is the chess-logic teammate's **v1.3.3.4** line (arm, board CAD, wrist/gripper, chess brain, coordinator) merged with the
M6 audit material (validator, static audit, Design-URDF review) and the first control stage (`chess_robot_executor`).
**Runtime behaviour of v1.3.3.4 is unchanged**; the executor is opt-in and off by default (see `CHANGELOG.md`).

> **Status (21 Oct 2026).** Built and run on a local ROS 2 Jazzy machine (terminal output as evidence): `colcon build` (6 packages),
> `tools/static_audit.py` OK (6 packages, 40 Python files, 4 xacro variants valid), `chess_robot_executor` tests 68 passed,
> the executor in dry-run (a full e2-e4 pick-and-place appeared in the log, nothing moved) and the original `simulation.launch.py`
> (Gazebo, RViz, bridge, brain, coordinator) start without errors.
> **Not verified locally yet:** the tests of the chess-logic packages and of `dual_chess_description` (checked in a non-ROS sandbox only),
> real (non dry-run) executor commands. v1.3.3.4 itself is a *candidate* (gripper steady-state tuning) according to its own notes (`docs/VERSION_HISTORY_v1.3.x.md`).

## 1. Packages

| Package | Type | Content |
|---|---|---|
| `dual_chess_description` | ament_cmake | URDF/Xacro (`dual_chess.urdf.xacro`), Design-team CAD meshes, supplied board CAD, temporary wrist/gripper meshes, RViz config, `display.launch.py` |
| `dual_chess_gazebo` | ament_cmake | world, ROS–Gazebo bridge (`bridge.yaml`), `gazebo.launch.py` |
| `dual_chess_bringup` | ament_cmake | `simulation.launch.py` (Gazebo + RViz + chess brain + passive coordinator), `chess_logic_only.launch.py`; opt-in `executor_enabled` |
| `chess_brain_handover` | ament_python | Stockfish-backed chess logic, JSON ROS interface (`/chess/*`), vendored python-chess |
| `chess_auto_coordinator` | ament_python | passive-by-default high-level coordinator (never drives joints) |
| `chess_robot_executor` | ament_python | **new:** move plans -> SCARA joint trajectories -> ack; pure-Python core + thin node; DRY-RUN by default; all arm geometry is placeholder |

Repository layout: `simulation/urdf_convert/` = untouched Design-team SolidWorks export (ROS 1 package, **do not** put it under `src/`);
`simulation/ROS2/dual_chess_ws/` = this workspace; `tools/` = checks and offline tools; `docs/` = documentation.

## 2. Requirements

```bash
# one-time (asks before installing; see the script): stockfish, ros_gz, xacro, rviz2, joint_state_publisher(_gui), colcon, pip
bash scripts/setup_system_dependencies.sh
```
The first `colcon build` downloads python-chess 1.11.2 into `chess_brain_handover/_vendor` (internet needed unless already present).
Optional developer tools: `pip install pytest pyyaml matplotlib` (executor tests and plots), `pip install yourdfpy trimesh python-fcl numpy` (`tools/model_checks.py`).

## 3. Build and run

```bash
cd ~/Chess_robotic_arm/simulation/ROS2/dual_chess_ws          # adjust to your path
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

ros2 launch dual_chess_bringup simulation.launch.py            # Gazebo + RViz + chess brain + passive coordinator
ros2 launch dual_chess_bringup chess_logic_only.launch.py      # chess logic only (no Gazebo/RViz)
ros2 launch dual_chess_description display.launch.py           # RViz + joint sliders only (no Gazebo)
ros2 launch chess_robot_executor executor.launch.py            # robot executor alone (dry-run: logs only, never acks)
```
Launch options: `rviz:=false`, `gz_args:="-r -s"` (headless), `robot1_enabled:=false`, `robot2_enabled:=false`, `chess_brain_enabled:=false`,
`auto_coordinator_enabled:=false`, `executor_enabled:=true` (+ `executor_dry_run:=false` only with the final arm model),
`auto_play_enabled:=true` (**leave false** until the executor drives a real/simulated arm), `stockfish_path:=…`.

## 4. Joint commands and the robot executor

### 4.1 Simulation joint commands (temporary stand-in, not the real controller)
Per robot (`robot1`, `robot2`): `/<robot>/base_yaw_joint/cmd_pos`, `/joint1/cmd_pos`, `/joint2/cmd_pos`, `/joint3/cmd_pos` (rad),
`/gripper_finger_joint/cmd_pos` (**metres**, 0–0.045). HOME: base_yaw π/2, joint1 1.15, joint2 0.35, joint3 0, gripper 0.
```bash
ros2 topic pub --once /robot1/joint3/cmd_pos std_msgs/msg/Float64 "{data: 0.1}"
ros2 topic echo /joint_states --once
ros2 topic echo /chess/state --once          # chess brain      ros2 topic echo /chess/auto/status --once   # coordinator (enabled=false)
```
Gripper acceptance test of v1.3.3.4: `docs/VERSION_HISTORY_v1.3.x.md` (top section).

### 4.2 Robot executor (`chess_robot_executor`, control stage 1)
Takes each move plan from the brain (`/chess/move_plan`), plans the pick-and-place (captures, en passant, castling, promotion with an operator step),
solves SCARA inverse kinematics, generates speed/acceleration-limited trajectories and acknowledges on `/chess/robot_ack`.
**Dry-run is the default**: commands are only logged, nothing moves, and no acknowledgement is sent.
```bash
ros2 launch dual_chess_bringup chess_logic_only.launch.py executor_enabled:=true         # brain + coordinator + executor (dry-run)
ros2 topic echo /chess/executor/status --full-length                                      # events: plan_started, plan_done, ack_suppressed_dry_run
ros2 topic pub --once /chess/move_plan std_msgs/msg/String "{data: '{\"plan_id\":\"t1\",\"side\":\"white\",\"piece\":\"P\",\"actions\":[{\"kind\":\"move\",\"source\":\"e2\",\"target\":\"e4\"}]}'}"
```
Use a new `plan_id` for every test plan (already executed ids are ignored). Software emergency stop: `ros2 topic pub --once /chess/executor/estop std_msgs/msg/Bool "{data: true}"`
(release with `false`, then `/chess/executor/recovered` once the arms were parked by hand).
The executor **cannot drive the current simulated arm** (not a SCARA: no Z axis); real commands (`dry_run:=false sink:=cmd_pos`) wait for the final model.
All its geometry, limits, speeds, piece heights and bin positions are **placeholders** (`src/chess_robot_executor/config/executor_params.yaml`).
Details: `docs/06_control_architecture.md`, `docs/INTEGRATION_CONTRACT.md`, `src/chess_robot_executor/README.md`.

## 5. What is Design data and what is temporary

| Part | Source |
|---|---|
| `link0`, `link1`, `link2`, `joint1`, `joint2` (axes, origins, CAD mass/inertia) | **Design-team export** `Assem4.SLDASM` — kinematics verified identical (max difference 0.0) |
| `base_yaw_joint` | added; described in the code as an intended motor axis — **confirm with Design** |
| `joint3` wrist, gripper, `tool0` | **temporary** (simulation geometry, illustrative mass/limits) |
| Board | supplied CAD (`Board_1_original.STL`): **40 cm frame, 35 cm playing area, 4.375 cm squares** — confirmed as the board to use for now |
| Final arm (SCARA-style, to be delivered) | **not available yet**; the executor's SCARA parameters are placeholders until then |
| Joint limits, motor torque/speed, collision meshes | **not defined yet** (continuous joints; primitive collision boxes) |

## 6. Open decisions / known gaps

1. **Board:** the supplied CAD board (35 cm playing area, 4.375 cm squares) is used for now; the physical board must still be confirmed by M2 (`urdf/board/board_params_cad.xacro`, executor `board.play_size`).
2. **Base yaw, wrist, gripper:** not in the Design URDF — confirm or replace with the final Design arm.
3. **Robot executor exists (dry-run) but cannot drive the current simulated arm:** `chess_robot_executor` is tested offline with the brain in a closed loop and was run locally in dry-run; real commands wait for the final SCARA model (`docs/06_control_architecture.md`). Still missing: `/chess/vision/board` (M7) and a physical safety source (the executor can publish a software `/chess/safety_ready` with `publish_safety_ready:=true`).
4. **Dual-arm safety:** at HOME both arms lie in the same vertical plane (x ≈ 0); no collision-aware planning, no joint limits for base yaw/joint1/joint2.
5. Chess pieces are not in Gazebo; vision is not connected; ros2_control and the real stepper interface are not started (they need the final model and M5/M9 data).
6. Licensing: vendored python-chess is GPL-3.0-or-later (`src/chess_brain_handover/THIRD_PARTY_NOTICES.md`).

## 7. Troubleshooting
- *Package not found:* `source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash`.
- *Mesh not found in Gazebo:* the launch appends the parent of the installed package folder to `GZ_SIM_RESOURCE_PATH`; export it yourself if you start `gz` manually.
- *Model not spawned / SDF frame errors:* first `[Err]` lines of the launch terminal; fixed-joint links without `<inertial>` are dropped by Gazebo (the board keeps one on purpose).
- *Build fails on python-chess:* needs internet once, or `pip install --target` into `~/.venvs/chess_ros2` as described in `setup.py`.
- *Gripper stuck at 0.045 m:* see the v1.3.3.x notes in `docs/VERSION_HISTORY_v1.3.x.md`.
- *Executor ignores my plan:* the `plan_id` was already used, or the executor is busy/in emergency stop (see `/chess/executor/status --full-length`).
- Qt "Binding loop", `libEGL`/`dri2` and `Component.hh ... operator<<` warnings from Gazebo are harmless on the tested laptop.

## 8. Checks (`tools/`)

```bash
python3 tools/static_audit.py                    # manifests, install rules, launch references, mesh URIs, xacro variants (needs xacro)
python3 tools/model_checks.py                    # needs: pip install yourdfpy trimesh python-fcl numpy
python3 tools/validate_design_urdf.py ../../urdf_convert/urdf/Assem4.SLDASM.urdf --mesh-root ../../urdf_convert   # check a new Design export
cd src/chess_auto_coordinator && python3 -m pytest -q tests ; cd ../chess_brain_handover && python3 -m pytest -q tests
cd ../chess_robot_executor && python3 -m pytest -q tests                      # 68 tests: kinematics, trajectories, planner, controller, closed loop with the brain
cd ../.. && python3 tools/simulate_executor.py --plot docs/figures/executor_tool_paths.png    # offline game through the executor (no ROS)
python3 tools/scara_reach_report.py --bins                                    # reach/joint-range requirements for the Design team
```
Results:
- **Run locally (21 Oct 2026):** `tools/static_audit.py` OK (6 packages, 40 Python files; xacro default / `sim_gazebo` / robot2 off / both off: 85, 85, 77 and 69 links); `chess_robot_executor` tests 68 passed.
- **Run in a non-ROS sandbox:** coordinator 12, brain 25, description 26 tests passed; Design chain identical to the Design URDF (max difference 0.0); HOME pose clear (arms 14.5 cm apart, no board collision); positional reach 64/64 squares (position only).
See `docs/05_workspace_audit.md`, `docs/model_checks_output.txt`, `docs/07_control_progress_report.md`.

## 9. Documentation
`docs/README.md` indexes everything: audit, control architecture, integration contract, control progress report, Design-URDF review, version history, and `archive_placeholder_arm/` (earlier Week 1 placeholder-arm work, kept for reference).
