# Control stage 1 overlay — how to add it to your v5 workspace

This ZIP contains only what is NEW or CHANGED relative to `Chess_Robot_M6_Merged_v5` (= your friend's v1.3.3.4 + the M6 audit material).
Paths are relative to the workspace root `simulation/ROS2/dual_chess_ws/`.

| Path | New / changed |
|---|---|
| `src/chess_robot_executor/` | **new package** (code, config, launch, 68 tests, README) |
| `src/dual_chess_bringup/launch/simulation.launch.py`, `chess_logic_only.launch.py` | **changed**: opt-in `executor_enabled` / `executor_dry_run` (default off) |
| `src/dual_chess_bringup/package.xml` | **changed**: `exec_depend chess_robot_executor` |
| `tools/simulate_executor.py`, `tools/scara_reach_report.py` | **new** |
| `docs/INTEGRATION_CONTRACT.md`, `06_control_architecture.md`, `07_control_progress_report.md`, `scara_reach_report_output.txt`, `simulate_executor_output.txt`, `figures/executor_tool_paths.png` | **new** |

## Install
```bash
cd ~/Chess_robotic_arm/simulation/ROS2/dual_chess_ws          # your v5 workspace
cp -r /path/to/overlay/src/. src/ && cp -r /path/to/overlay/tools/. tools/ && cp -r /path/to/overlay/docs/. docs/
source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash
(cd src/chess_robot_executor && python3 -m pytest -q tests)   # 68 passed expected
python3 tools/static_audit.py                                  # needs xacro; expect: OK - no problems found
ros2 launch dual_chess_bringup chess_logic_only.launch.py executor_enabled:=true   # dry-run
```

## Optional text edits (copy-paste)
**README.md** (packages table): add a row  
`| chess_robot_executor | ament_python | move plans -> SCARA joint trajectories -> ack; DRY-RUN by default; all geometry placeholder |`  
and in §6 replace open item 3 by: *"Robot executor exists (`chess_robot_executor`, dry-run by default, tested offline with the brain in a closed loop) but cannot drive the current simulated arm because it is not a SCARA; real commands wait for the final model (`docs/06_control_architecture.md`). Still missing: `/chess/vision/board` (M7) and a physical safety source."*  
**CHANGELOG.md**: add at the top "Control stage 1: new package `chess_robot_executor` (68 tests), opt-in `executor_enabled` in bringup, tools and docs; see `docs/07_control_progress_report.md`."  
**docs/README.md**: add rows for `06_control_architecture.md`, `INTEGRATION_CONTRACT.md`, `07_control_progress_report.md`.

If `colcon build` complains about the new package, send me the first error lines.
