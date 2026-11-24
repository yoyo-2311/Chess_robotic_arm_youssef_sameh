# Local Dual SCARA desktop

`dual_scara_desktop` is a PySide6/rclpy application for this Ubuntu 24.04 / ROS 2 Jazzy workspace. It launches the existing backends and uses their ROS interfaces. No web server, HTTP interface, camera algorithm, game rules, hardware protocol or replacement vision source is added.

## Install, build and start

Run these commands from `Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws`:

```bash
source /opt/ros/jazzy/setup.bash
# Existing ROS/MoveIt/Gazebo dependencies:
./scripts/setup_system_dependencies.sh
# Stockfish and native Qt X11 library, if not already installed:
sudo apt-get install stockfish libxcb-cursor0 ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers
# Installs PySide6 6.8.3 under this workspace's ignored .qt-deps directory.
# It can also download/extract libxcb-cursor0 locally without sudo.
./scripts/install_desktop_dependencies.sh
colcon build --symlink-install
./scripts/run_desktop.sh
```

The launcher works from any current directory and sources Jazzy and this workspace. The documented ROS entry point is also available:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run dual_scara_desktop desktop --workspace "$PWD" --ros-domain-id 211
```

Use the system Python associated with Jazzy, not a virtual environment that hides `rclpy`. PySide6 is loaded from `.qt-deps`; it is not installed into system Python. Installation needs internet access. Native Qt needs a logged-in graphical desktop (X11 or Wayland); `QT_QPA_PLATFORM=offscreen` is for automated tests. RViz and Gazebo have their own windows. The Stockfish executable defaults to `/usr/games/stockfish` and is editable before Start.

## First match

1. Select **Human vs Stockfish — simulated match**. Leave the supplied geometry profile selected. Choose White or Black. Optionally enable RViz.
2. Press **Start mode**. Wait for “Confirmed board ready”. Startup includes controllers, collision scene, initial mock vision sync, and readiness checks; buttons remain disabled until these complete.
3. For White, click `e2` then `e4`, or enter `e2e4` and press Submit. White always uses **robot1**, Black **robot2**. If you choose Black, exactly one White opening move is requested automatically; wait for its confirmed completion before selecting your move.
4. Watch progress, pending move, execution result and the log tab. After successful trajectory execution, the robot acknowledgement still does **not** change the board. A fresh matching observation from the existing mock vision must commit it in Chess Brain. Only then does the desktop request exactly one Stockfish reply.
5. After the reply has executed and been vision-confirmed, human input is restored. History contains confirmed moves with the executing arm. A promotion can require the backend's operator confirmation; read and confirm the displayed request. No board update is fabricated by the GUI.
6. **Stop mode** or close the window to stop the owned launch and children. Closing waits asynchronously for shutdown. A new simulated session starts from its launch FEN; game persistence/resuming an existing backend is deliberately unsupported.

The supplied fixed-gripper CAD and geometry remain simulation placeholders. A legal chess move can fail a collision or Cartesian-path check. For example, one tested Black `e7e5` reply after White `d2d4` failed the existing Cartesian clearance check; the confirmed position stayed unchanged and automatic progression paused. The application does not bypass collision checks to make a match continue. The verified Black walkthrough used `g8f6` after Stockfish's `d2d4`.

## Modes and controls

| Mode | Controls and execution meaning |
|---|---|
| Manual joints — visualization | Select robot1/robot2; sliders and numeric fields use model limits, metres for the prismatic joint and radians for revolute joints. The desktop is the only joint-state publisher. Robot state publisher supplies TF and optional RViz displays movement. No planning, collision protection, motor command or physics is implied. |
| MoveIt / square tests | Select arm and square. **Plan** asks the backend for a path without moving joints. **Execute** computes a new path and executes it through mock ros2_control trajectory controllers; it does not execute a cached Plan. **Home** plans/executes home. **Empty-square probe** requires confirmation that the square is empty, then approaches, descends to configured pawn grasp height and retreats. Planning and execution results are separate. RViz's existing MotionPlanning plugin can also be used; avoid issuing competing RViz/CLI commands while a desktop request is running. |
| Gazebo | Existing Gazebo Harmonic physics launch with direct joint position topics and clock/joint feedback. Select an arm, set values, then Apply. This launch has no MoveIt trajectory-controller bridge: these controls do not claim MoveIt execution. The CAD/floor/contact model is provisional. `--headless-gazebo` runs the server without its window. |
| Full mock chess | Existing deterministic mock engine, mock gripper, mock vision and MoveIt mock controllers. Submit either side's legal UCI moves. Select a scenario **before** starting: Standard; Capture `e4d5`; Castling `e1g1`; En passant `e5d6`; Promotion `a7a8q`. The existing Chess Brain expands multi-piece operations and validates rules. Unlike Stockfish match mode, there is no automatic opponent. |
| Human vs Stockfish | Existing Stockfish-backed `next` command, same mock motion/gripper/vision pipeline, one desktop turn owner. Human color selection changes who submits moves, never the arm/color assignment. |
| Chess logic only | Existing brain launch, with its automatic coordinator and legacy executor disabled. Shows brain state and supports state/sync requests. There is no bundled vision or safety source; initial sync reports missing vision and motion input stays disabled. Use the full mock mode for a self-contained playable pipeline. |

Only one app-managed mode may run per selected ROS domain. Switching stops it before starting the next. The app refuses an occupied domain, checks duplicate publishers/coordinators, and does not kill independently started processes. It sets ROS discovery to localhost, clears static remote peers, and uses a private localhost Gazebo partition. It does not attach to or resume a previously running match. Choose another `--ros-domain-id` only for an intentionally isolated test.

## Failure, confirmation and recovery

- **Fail next gripper operation** uses the existing mock gripper parameter. Wait for its acknowledgement. A subsequent execution failure pauses progression and leaves confirmed FEN/history unchanged.
- **Hold vision confirmation** disables the existing mock vision's post-ack confirmation. A completed trajectory/robot acknowledgement remains pending; releasing the checkbox allows the mock to send a fresh observation. Parameter changes are asynchronous and block new moves until acknowledged.
- **Confirm operator request** uses the exact current plan/token from the backend. Repeated clicks cannot confirm another request. This preserves the existing promotion replacement flow; the GUI does not invent an automatic physical promotion mechanism.
- **Software motion stop** publishes the existing latched software stop and immediately disables commands. It keeps the mode running for inspection. This is distinct from **Stop mode**, which shuts down app-managed processes. It is not a hardware emergency stop.
- After a failure, stop, lost heartbeat, unrecognized confirmed transition or competing turn owner, automatic play stays paused. **Restart simulated session** requires explicit confirmation, stops the owned mode and starts a fresh simulated board. It does not reconcile or resume a physical board. For hardware, reconcile physical/vision state and use the backend's documented recovery process before re-enabling motion.

## Geometry profiles and physical revalidation

Use **Browse** to select a YAML profile before Start. The default is the installed `dual_chess_description/config/chess_geometry.yaml`. The app validates it with the existing `Geometry` class and passes its absolute path as `geometry_file` to supported launches (all bundled modes except logic-only). Paths containing spaces are passed as single arguments, without a shell. Effective board/mount configuration and revision are displayed from `/chess/geometry` where available.

Physical dimensions, pose, height, square pitch, arm mounting transforms, piece heights, TCP offsets and square-to-world conversion belong to the existing YAML/ROS backend. The Qt board only rescales pixels and displays confirmed FEN. Resize the window freely; this never changes physical dimensions. To change the physical board, stop the mode, edit/copy the profile, select it and restart. Do not edit a live profile expecting running nodes to reload it.

Revalidate all 64 squares for both arms, low-surface Z reach, joint limits, approach/grasp/retreat clearance, support/table collisions, pieces, capture bins, arm interference, homing, controller timing and gripper/TCP calibration after changing geometry. The initial surface is only 10 mm above its support; arm base origins are separately configured. These are placeholders, not measured hardware specifications. A small simulation pad does not prove clearance over an arbitrary real table. See [the motion-interface guide](09_configurable_motion_interfaces.md) for geometry audits and backend test commands.

## Ownership and integration

Widgets in `window.py` only collect input and show state. `controller.py` gates controls and coordinates mode lifecycle. `coordinator.py` is a ROS/Qt-independent state machine; it submits through Chess Brain's advertised legal moves and never calculates authoritative game state. `ros_bridge.py` owns rclpy in a worker thread with its own executor/context; `supervisor.py` monitors subprocesses in another worker, tracks owned descendants, uses a per-domain lock and escalates shutdown only for those processes. Qt's timer drains queues without waiting for ROS callbacks or process termination.

`stockfish_match.launch.py` reuses the existing MoveIt demo, Chess Brain, mock gripper and mock vision, without starting a competing coordinator. `manual.launch.py` reuses the description and RViz configuration without joint_state_publisher_gui because Qt owns this mode's single publisher. Existing launches retain their defaults; an optional `motion_start_delay` is used by desktop modes to let controller/TF discovery settle.

Existing topics/schemas remain unchanged: `/chess/command`, `/chess/state`, `/chess/events`, `/chess/move_plan`, `/chess/robot_ack`, `/chess/vision/board`, `/chess/safety_ready`, `/chess/motion/{square_command,status,operator_confirm,estop}` and `/chess/geometry`. A future vision implementation integrates through those backend interfaces; it must provide fresh observations and preserve Chess Brain confirmation authority. Future embedded integration belongs behind ros2_control/gripper adapters after hardware specifications are supplied. This application adds no hardware assumptions.

## Repeatable checks

From the workspace after build:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/.qt-deps:$PYTHONPATH"
export QT_QPA_PLATFORM=offscreen
python3 -m pytest -q src/dual_scara_desktop/tests
python3 src/dual_scara_desktop/tests/gui_match_probe.py --workspace "$PWD" --domain 212 --human white
python3 src/dual_scara_desktop/tests/gui_match_probe.py --workspace "$PWD" --domain 213 --human black
python3 src/dual_scara_desktop/tests/gui_modes_probe.py --workspace "$PWD" --domain 214
colcon test --python-testing pytest --event-handlers console_direct+
colcon test-result --verbose
python3 tools/static_audit.py
```

The explicit GUI probes drive actual Qt controls and ROS processes; they are not default pytest tests. They create ignored `.desktop-state` JSON/screenshots and launch logs. Tests cover both human colors, duplicate input/confirmation, delayed vision, failed motion, process ownership and mode transitions. See `desktop_validation/verification.md` for checks actually run and remaining limits.

Complete-game checks, a replayable Stockfish PGN, terminal-state logic tests and the motion limitation found during unrestricted play are documented in [full-game validation](full_game_validation/README.md).

For opening-move collision preflight, explicit side/arm progress, timeout handling and the verified `g1` grasp limitation, see [move-stall diagnosis and regression results](11_move_stall_diagnosis.md).

The follow-up [exact-position pickup diagnosis and safe-rejection recovery](12_pickup_geometry_and_recovery.md) includes point-by-point contacts after `c4 e5`, a separately validated upper-grasp simulation profile, and human-turn recovery after proven pre-activity rejection.
