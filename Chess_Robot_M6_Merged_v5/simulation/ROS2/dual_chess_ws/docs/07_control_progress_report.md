# Control stage 1 — progress report (19 Oct 2026)

## 1. Summary
The control chain between the chess brain and the arms now exists as a new package, **`chess_robot_executor`**: it takes each move plan from the brain, plans the pick/place sequence, solves SCARA inverse kinematics, generates speed- and acceleration-limited trajectories, and acknowledges the result back to the brain. It is **tested offline** (68 tests, including whole games in a closed loop with the brain's own game controller) but **not yet run on a ROS machine**, and it cannot drive the current Gazebo arm because that arm is not a SCARA. Everything about the arm geometry is a placeholder until the Design team delivers the final design. The default mode is dry-run: nothing moves and no acknowledgement is sent.

## 2. What was built
| Component | What it does |
|---|---|
| `board_geometry`, `scara` | square centres from the board CAD (35 cm playing area, 4.375 cm squares); analytic SCARA FK/IK with joint, Z and reach limits |
| `trajectory` | all axes start and stop together; trapezoidal velocity profiles within per-axis velocity and acceleration limits |
| `task_planner` | move plan (`remove` / `move` / `promote`) to pick-and-place steps: captures and en passant, castling (king then rook), promotion with an operator step, per-arm capture bins (24 slots) |
| `controller` | state machine: pre-flight reachability and limit check, one arm at a time, estop, operator confirmation and timeout, recovery, duplicate/busy plan handling, acknowledgement |
| `node`, `executor.launch.py`, `executor_params.yaml` | thin ROS 2 node (topics, timers, parameters), launch file, one configuration file with every placeholder labelled |
| bringup | opt-in `executor_enabled` in both launch files (default off, existing behaviour unchanged) |
| tools and docs | `simulate_executor.py` (offline game and tool-path plot), `scara_reach_report.py`, `INTEGRATION_CONTRACT.md`, `06_control_architecture.md` |

## 3. Safety rules built in (and tested for every kind of move)
Horizontal travel only at the travel height above the tallest piece; vertical moves only in Z; the gripper only moves while the arm is still; only the assigned arm is commanded; every target is checked before any motion; estop aborts at once and blocks plans until an operator confirms recovery. These are software rules, not a physical safety interlock.

## 4. Verification (sandbox, no ROS)
| Check | Result |
|---|---|
| Executor tests | 68 passed: kinematics/geometry 12, trajectories 5, planner 21, controller 21, closed loop 4, node wiring 5 |
| All 64 squares and all 24 bin slots reachable by both arms (placeholder geometry) | yes |
| Every legal move of the start position and of castling/promotion test positions | executed and acknowledged |
| Random full games, brain `GameController` + executor + simulated camera, 3 seeds | 60 to 120 plies each, no failure |
| Offline simulation, 60 plies | about 21 s per move at `speed_scale` 0.5, longest about 36 s (placeholder speeds) |
| ROS node | exercised against a stub of `rclpy` only: **needs a real run** |

## 5. Findings along the way
- The brain's `INTEGRATION_CONTRACT.md` is missing from the archive; I reconstructed it from the code (`docs/INTEGRATION_CONTRACT.md`).
- Nothing publishes `/chess/safety_ready`, so the brain would never issue a plan: the executor can now publish it (opt-in, software state only).
- A 16-slot capture bin is too small (up to 15 captured pieces plus 8 promoted pawns): now 24.
- The static audit caught two undeclared launch dependencies in my new package; fixed.
- Joint-space travel from the park pose makes wide arcs (see `docs/figures/executor_tool_paths.png`); whether that is safe depends on the real arm geometry.
- Reach analysis for the Design team: with the base 30 cm from the board centre the farthest square is 47.8 cm away (arm needs l1+l2 of at least 52.6 cm with 10 % margin) and the nearest 14.8 cm (|l1-l2| must be below that); the nearest squares force an elbow fold of about 150 degrees. Moving the base further out reduces the fold but needs more reach.

## 6. Not done
ros2_control integration and the real stepper interface (both need the final model and the M5/M9 data); collision-aware planning and a physical safety interlock; chess pieces and a camera in Gazebo; vision interface; hardware feedback (acknowledgements come from commanded state); a run on a ROS 2 machine.

## 7. Next steps
1. Run it on your machine (README of the executor package): dry-run first, then check the status topic.
2. When the Design team sends the final arm: define joint names and limits, update `executor_params.yaml`, re-run the reach report and the tests.
3. Replace the temporary Gazebo controllers with ros2_control and add a `FollowJointTrajectory` sink (the first real feedback).
4. Agree with M5/M9 on the stepper interface, homing and e-stop; with M7 on the vision interface; with M2 on piece data and the grasp.
