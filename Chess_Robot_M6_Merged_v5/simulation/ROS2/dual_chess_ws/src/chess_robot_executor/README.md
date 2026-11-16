> **SCARA integration (16 Nov 2026):** This package is retained as the original offline planar prototype. Its geometry and board settings describe the old example, not the active CAD scene. Live mode is disabled. Use `dual_chess_moveit_config` for CAD motion; see `docs/08_scara_moveit.md`.

# chess_robot_executor

Turns the chess brain's `/chess/move_plan` into SCARA joint trajectories and acknowledges on `/chess/robot_ack`.
**Every geometry/limit value is a placeholder until the Design team delivers the final arm. Default mode is DRY-RUN: nothing moves and no ack is sent.**
Details: `docs/06_control_architecture.md`, `docs/INTEGRATION_CONTRACT.md`.

```bash
ros2 launch chess_robot_executor executor.launch.py                         # dry-run (logs only)
ros2 launch dual_chess_bringup chess_logic_only.launch.py executor_enabled:=true      # brain + coordinator + executor (dry-run)
# send a test plan (the brain normally does this):
ros2 topic pub --once /chess/move_plan std_msgs/msg/String "{data: '{\"plan_id\":\"t1\",\"side\":\"white\",\"piece\":\"P\",\"actions\":[{\"kind\":\"move\",\"source\":\"e2\",\"target\":\"e4\"}]}'}"
ros2 topic echo /chess/executor/status
```
Parameters: `config_file`, `dry_run` (true), `sink` (`log`|`cmd_pos`; `cmd_pos` only with `dry_run:=false`), `ack_in_dry_run` (false), `speed_scale` (override, -1 = config), `publish_safety_ready` (false), `tick_hz` (50), `status_hz` (2).
Offline checks (no ROS): `cd src/chess_robot_executor && python3 -m pytest -q tests`; `python3 tools/simulate_executor.py --plot docs/figures/executor_tool_paths.png`; `python3 tools/scara_reach_report.py --bins`.
