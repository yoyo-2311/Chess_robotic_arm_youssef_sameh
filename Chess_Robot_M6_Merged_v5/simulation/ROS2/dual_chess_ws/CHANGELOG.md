# Changelog

## Control stage 1 (executor, kinematics, trajectories) — 21 Oct 2026
### Added
- `src/chess_robot_executor`: pure-Python core (board geometry, analytic SCARA IK/FK, synchronised trapezoidal trajectories, move-plan task planner, executor state machine with emergency stop / operator confirmation / recovery) + ROS 2 node (`robot_executor`), `config/executor_params.yaml` (all placeholders), `launch/executor.launch.py`. Dry-run by default; no acknowledgement is sent in dry-run.
- 68 tests: kinematics, trajectories, planner, controller safety invariants, full random games, closed loop with the brain's `GameController`, node wiring against an rclpy stub.
- `tools/simulate_executor.py` (offline game + tool-path plot), `tools/scara_reach_report.py` (reach/joint-range requirements for the Design team).
- `docs/INTEGRATION_CONTRACT.md` (reconstructed: the brain's original was missing), `docs/06_control_architecture.md`, `docs/07_control_progress_report.md`, outputs and `docs/figures/executor_tool_paths.png`.
- `dual_chess_bringup`: opt-in `executor_enabled` / `executor_dry_run` in `simulation.launch.py` and `chess_logic_only.launch.py` (default off, existing behaviour unchanged); `exec_depend chess_robot_executor`.
### Verified locally (author, Ubuntu 24.04 / ROS 2 Jazzy)
- `colcon build` (6 packages), `tools/static_audit.py` OK, executor tests 68 passed, executor dry-run (e2-e4 in the log, nothing moved), original `simulation.launch.py` unchanged and error-free.
### Not done / limitations
- No real (non dry-run) commands: the current simulated arm is not a SCARA. No hardware feedback, no collision checking, no physical safety interlock, ros2_control and stepper interface not started (need the final arm model and M5/M9 data). See `docs/06_control_architecture.md`.

## Merge of v1.3.3.4 with the M6 audit material
### Added
- `tools/static_audit.py` (cmake + python packages), `tools/model_checks.py`, `tools/validate_design_urdf.py`.
- `docs/` index, `05_workspace_audit.md`, `04_design_urdf_review.md` (+ status banner), `model_checks_output.txt`, `design_urdf_validation_output.txt`.
- `simulation/urdf_convert/` (original Design export) and `simulation/README.md`.
- `src/chess_brain_handover/THIRD_PARTY_NOTICES.md` (vendored python-chess is GPL-3.0-or-later).
- `docs/archive_placeholder_arm/` — earlier placeholder-arm docs, Week 1 report, v4 audit changelog (historical).
### Fixed (no runtime behaviour change)
- Stale `package.xml` descriptions of `dual_chess_description`, `dual_chess_gazebo`, `dual_chess_bringup`.
- `chess_brain_handover/tests/test_chess_brain.py`: `import chess` failed unless python-chess was installed separately; now falls back to the vendored copy.
- Root README rewritten as a user guide; the original v1.3.x notes moved unchanged to `docs/VERSION_HISTORY_v1.3.x.md`.
### Not changed
- All URDF/Xacro, meshes, Gazebo world/bridge, chess packages, scripts. Board remains the supplied CAD (35 cm playing area).

## v1.3.3.4 and earlier (by the chess-logic teammate)
See `docs/VERSION_HISTORY_v1.3.x.md`.
