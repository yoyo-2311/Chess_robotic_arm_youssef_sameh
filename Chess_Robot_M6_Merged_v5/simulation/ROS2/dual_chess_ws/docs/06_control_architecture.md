# Control architecture and status of the control stages

## 1. Layers
```
 chess brain (Stockfish, rules)  ->  /chess/move_plan  ->  [ robot executor ]  ->  /chess/robot_ack  -> brain
 vision (M7, not provided) ------>  /chess/vision/board ---------------------------------------------> brain
                                              executor = task planner -> IK (SCARA) -> trajectories -> sink
                                                                                              |
                          sink 'log' (dry-run, default) | sink 'cmd_pos' (Float64 per joint) | future: FollowJointTrajectory (ros2_control)
                                                                                              v
                                          Gazebo plugins now -> gz_ros2_control (sim) -> real stepper drivers (M5/M9)
```
The executor is `src/chess_robot_executor`: a pure-Python core (`board_geometry`, `scara`, `trajectory`, `task_planner`, `controller`, `sinks`) plus a thin ROS 2 node (`node.py`). The core has no ROS dependency, so it is unit-tested without ROS; the node only wires topics and timers.

## 2. Stage status
| # | Stage | Status |
|---|---|---|
| 1 | Interface contract brain <-> executor | **Done** - `INTEGRATION_CONTRACT.md` |
| 2 | Robot executor (plan -> steps -> joint commands, state machine, estop, operator step, bins, dry-run) | **Implemented**, 68 offline tests; ROS node verified only against a stub of rclpy; **not yet run on a ROS machine** |
| 3 | SCARA kinematics (analytic FK/IK, limits, reach margin) | **Implemented** with PLACEHOLDER geometry; final values from the Design team |
| 4 | Trajectory generation (synchronised trapezoidal profiles, rate/acceleration limited) | **Implemented** and tested (limits, endpoints, synchronisation) |
| 5 | Closed loop brain + executor + simulated camera | **Tested offline** (full random games, 3 seeds, 60-120 plies each) |
| 6 | ros2_control (`gz_ros2_control`, trajectory controllers) | **Not started** - needs the final model (joint names/limits); plan in section 6 |
| 7 | Real stepper interface (homing, limits, e-stop, step conversion) | **Not started** - needs M5/M9 data (section 7) |
| 8 | Dual-arm safety | **Partly**: one arm at a time, idle arm parked, software estop/recovery. **Missing**: collision-aware planning, physical interlock |
| 9 | Pieces and camera in Gazebo | **Not started** |

## 3. Behaviour and decisions
- **Arm choice** by colour (white -> robot1, black -> robot2); every target of the plan is checked with IK and joint/z limits **before** any motion; a failing plan is acknowledged as failed and nothing moves.
- **Motion structure** (a pick = raise to travel height, travel above the piece, open, descend, close, lift; a place = travel, descend, open, retreat): horizontal travel only at the travel height (above the tallest piece + clearance), vertical moves only in Z, the gripper only moves while the arm stands still. These invariants are asserted in the tests for every kind of move.
- **Trajectories**: all axes start/stop together; per-axis velocity and acceleration limits (placeholder values, `speed_scale` 0.5 by default).
- **Captures** go to a per-arm bin grid beside the board (24 slots: up to 15 captured pieces + 8 promoted pawns); counters reset when a plan starts from the initial position.
- **Promotion**: the pawn goes to the bin and the executor waits for an operator to place the new piece (no reserve of pieces exists yet).
- **Dry-run is the default**: commands are only logged and no ack is sent.
- **Failure handling**: estop aborts at once, acks failure (brain blocks) and refuses plans until `estop` is released **and** an operator confirms recovery (`/chess/executor/recovered`); operator timeout behaves the same.

## 4. Configuration (`config/executor_params.yaml`)
Every value is a PLACEHOLDER unless noted: board (supplied CAD: 35 cm playing area), mounts (current layout: robot1 at -Y facing +Y), SCARA link lengths (0.30 / 0.24 m, chosen only so that all squares are reachable), joint and z limits, speeds/accelerations, park pose, piece heights, gripper widths, capture bins, timing, joint names of the `cmd_pos` sink.

## 5. Offline checks and what they show (sandbox, no ROS)
| Check | Result |
|---|---|
| Unit/integration tests of the package | 68 passed (geometry/kinematics 12, trajectory 5, planner 21, controller 21, closed loop 4, node wiring 5) |
| All 64 squares reachable by both arms at every working height; all 24 bin slots reachable and outside the board | yes (placeholder geometry) |
| Every legal move of the start position and of castling/promotion test positions; en passant and castling of both colours in dedicated tests | all executed, acked as success |
| Random full games (3 seeds) through brain -> executor -> simulated vision | no failure |
| `tools/simulate_executor.py` (60 plies, seed 1) | about 21 s per move at `speed_scale` 0.5, longest about 36 s (placeholder speeds/accelerations) |
| `tools/scara_reach_report.py` | with the base 30 cm from the board centre the farthest square is 47.8 cm away (need l1+l2 >= 52.6 cm with 10 % margin) and the nearest 14.8 cm (need \|l1-l2\| <= 14.8 cm); the nearest squares force an elbow fold of about 150 deg |

## 6. Plan for ros2_control (when the final model exists)
1. Define joint names and limits from the Design URDF (e.g. `<arm>_shoulder_joint`, `_elbow_joint`, `_z_joint`, gripper); mimic joints only if the gripper mechanics require it.
2. Add `<ros2_control>` blocks (`gz_ros2_control/GazeboSimSystem` in simulation, a stepper hardware plugin later; a `mock_components` variant for CI) and remove the temporary Gazebo hold plugins.
3. One `joint_trajectory_controller` per arm (+ a gripper controller), namespaces `/robot1`, `/robot2`, loaded by a spawner launch.
4. New executor sink `trajectory_action`: send each synchronised segment as a `FollowJointTrajectory` goal (sampled waypoints with `time_from_start`) and use the action result/feedback for the ack - this is the first real feedback.
5. Keep the pure-Python core unchanged; only the sink and the config change.

## 7. What M5/M9 must provide for the real hardware
Stepper models, microstepping and drivers; steps-per-radian for the shoulder/elbow (gear/belt ratios) and steps-per-metre for the Z axis (lead-screw pitch); maximum speed/acceleration per axis; homing method and switch positions; end limits; gripper actuator and force/position feedback; e-stop wiring; communication link (serial/CAN/GPIO) and control rate.

## 8. Known limitations
- **No feedback**: acks come from commanded state; no encoders, switches or gripper sensing; the brain's vision has to verify each move.
- **Not a safety system**: software estop and `safety_ready` are software state, not an interlock.
- **No collision checking** against the base column, links, pieces or the other arm; the idle arm is only assumed parked. Joint-space travel from the park pose makes wide arcs (see `docs/figures/executor_tool_paths.png`): the real geometry decides whether they are safe, and Cartesian or via-point travel may be needed.
- **Placeholders everywhere** (section 4); timings will change with the real arm.
- **The temporary Gazebo arm is not a SCARA** (base yaw + 3 joints + gripper, no Z axis): the executor cannot drive it; with the `log` sink it only prints commands. It will drive the final model once joint names/limits exist.
- The ROS node was exercised against a stub only.

## 9. Open questions
| Question | Owner |
|---|---|
| Final arm: link lengths, joint ranges/limits, Z stroke, gripper, TCP, zero pose, forward direction | Design team (request document sent) |
| Base distance from the board and heights (reach analysis in `docs/scara_reach_report_output.txt`) | Design / M4 |
| Piece dimensions and grasp strategy | M2 |
| Motor/driver data, homing, limits, feedback | M5 / M9 |
| Who publishes `/chess/safety_ready` and how the physical e-stop is wired | M5 / M10 |
| Vision interface and calibration | M7 |
| Promotion handling (reserve pieces or operator) and capture bin location | team decision |
