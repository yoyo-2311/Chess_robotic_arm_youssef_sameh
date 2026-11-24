# Dual SCARA Chess Robot — start here

Local **PySide6 Qt desktop + ROS 2 Jazzy + MoveIt 2**, with Gazebo Harmonic,
Stockfish and existing mock gripper/vision interfaces. Robot1 plays **White**;
robot2 plays **Black**. This guide is for a teammate starting on another laptop.

## Current project status — read before choosing a profile

The source, CAD assets, Qt app, launch files, tests and validation reports are
in this repository. Generated `build/`, `install/`, launch logs and local Qt
libraries are intentionally excluded; install/build them on each laptop.

| Geometry profile, relative to the ROS workspace | What it represents | What to expect |
|---|---|---|
| `src/chess_workcell_description/config/workcell_sim.yaml` | New platform; physical base faces seated on measured A/B circles | Visualization, collision-free home, mock-controller home and Gazebo feedback verified. **All 64 board squares on each arm are unreachable with the supplied vertical travel. Chess moves reject safely.** Arm-to-seat identity and physical yaw remain unconfirmed previews. |
| `src/dual_chess_description/config/chess_geometry_adaptive_sim.yaml` | Earlier robot-only simulation, without the new platform | Use for software motion experiments. Recorded `c4 e5 Nf3 Nc6` execution/vision confirmation passed. This is not a physical mounting design or a guarantee that every move/full game is feasible. |
| `src/dual_chess_description/config/chess_geometry.yaml` | Original/default robot-only profile, fixed preferred grasp | Preserved for compatibility. Some pickups collide and reject. This is Qt's default if no profile is specified. |

For the new platform the TCP minimum Z is about **0.347 m**, while `g1` pickup
needs **0.0243 m**. Software installation or changing yaw cannot resolve this
vertical mechanical deficit. No collision/vision checks have been bypassed.
See [measured mounts and current validation](Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws/docs/14_mount_alignment.md).

## 1. Prepare a new laptop

Target: **Ubuntu 24.04, ROS 2 Jazzy, system Python 3.12**, with a graphical login
for Qt/RViz. Use Gazebo **Harmonic**, installed through `ros-jazzy-ros-gz`.
Windows/macOS, other ROS distributions, physical motors and a real camera are
not validated by this guide. No real robot is needed for the mock modes.

First install/configure ROS 2 Jazzy using the official
[Ubuntu deb installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
The commands below assume `/opt/ros/jazzy/setup.bash` exists and the ROS apt
repository is configured. Gazebo's [ROS compatibility/install guide](https://gazebosim.org/docs/harmonic/ros_installation/)
confirms the Jazzy/Harmonic pairing.

Install prerequisites once, as a normal user:

```bash
sudo apt update
sudo apt install -y \
  git curl python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-yaml python3-pytest \
  stockfish libxcb-cursor0 \
  ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui
```

## 2. Download the correct branch and build

```bash
mkdir -p "$HOME/projects"
cd "$HOME/projects"
git clone --branch configurable-chess-motion --single-branch \
  https://github.com/yoyo-2311/Chess_robotic_arm_youssef_sameh.git dual-scara-chess
cd dual-scara-chess/Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws

source /opt/ros/jazzy/setup.bash
# Initialize rosdep only if this machine has not already initialized it.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y

# PySide6 6.8.3 is installed inside this workspace's ignored .qt-deps directory.
./scripts/install_desktop_dependencies.sh
colcon build
source install/setup.bash
```

Internet is needed for apt/rosdep, the local PySide6 installation and the first
build's pinned `python-chess` dependency. `chess_brain_handover` provisions that
dependency locally; do not use `sudo pip`. Prefer the Jazzy system Python over
a venv/Conda environment that hides `rclpy`. The normal build above is the
validated path; rebuild after source/config changes instead of editing `install/`.

## 3. Start the Qt desktop

Run from the workspace directory used above:

```bash
# New platform, measured seated mounts — board movement currently unreachable.
./scripts/run_desktop.sh --ros-domain-id 211 \
  --geometry-file "$PWD/src/chess_workcell_description/config/workcell_sim.yaml"
```

For an **earlier robot-only software motion experiment**, close the app and use:

```bash
./scripts/run_desktop.sh --ros-domain-id 211 \
  --geometry-file "$PWD/src/dual_chess_description/config/chess_geometry_adaptive_sim.yaml"
```

The wrapper sources Jazzy and the current overlay automatically. Equivalent
ROS entry point, from a sourced workspace:

```bash
ros2 run dual_scara_desktop desktop --workspace "$PWD" --ros-domain-id 211 \
  --geometry-file "$PWD/src/chess_workcell_description/config/workcell_sim.yaml"
```

Start with **Manual joints — visualization only**, then press **Start mode**.
Wait for readiness. Select robot1/robot2 and move the sliders; units are metres
for joint1 and radians for joint2/3. Enable RViz before starting if desired.
Manual movement is visualization, not collision-protected execution.

### Qt mode selector

| Mode | How to use it | Meaning/limit |
|---|---|---|
| Manual joints | Select arm; sliders/numeric fields | Desktop owns the sole joint-state publisher. |
| MoveIt / square tests | Select arm/square, then Plan, Execute or Home | Plan does not move. Execute replans and sends a trajectory to mock ros2_control. Empty-square probe is only for an actually empty square. Seated-board targets reject; home works. |
| Gazebo | Start, select arm, set joints, Apply | Direct physics controls and feedback, not MoveIt chess execution. |
| Full mock chess | Select scenario before Start, then submit UCI or click source/destination | Deterministic engine, mock vision/gripper/controllers; no automatic opponent. |
| Human vs Stockfish | Choose human White/Black; click a legal move or enter UCI; choose promotion piece | Robot1=White, robot2=Black. Exactly one reply is requested only after successful execution **and fresh vision commit**. With the seated profile, moves reject and a match cannot complete. |
| Chess logic only | Start, inspect state/sync | No bundled vision/safety/motion; external integration is needed. |

For a software match on the earlier adaptive profile: choose Human vs Stockfish,
White, Start, then try `c2c4`. Wait for confirmed execution/vision and one reply
before another click. For Black, wait for one confirmed White opening move.
Stockfish defaults to `/usr/games/stockfish`; edit the field if installed elsewhere.
A planning failure remains a failure; no complete-game success is promised.

The scenario selector includes the exact `After c4 e5: g1f3` position, capture,
castling, en passant and promotion. Scenario initialization is mock simulation,
not a fabricated successful robot move. On the seated profile these scenarios
are useful for testing explicit, safe reach rejection rather than execution.

**Stop mode** shuts down only app-owned processes; a switch waits for shutdown.
**Software motion stop** blocks movement but keeps the backend running. Hold
vision, fail-next-gripper and token-based promotion confirmation are existing
mock controls. A robot acknowledgement alone never changes confirmed FEN.
After partial/uncertain execution, follow recovery/reconciliation; do not resend
blindly. See [desktop controls and recovery details](Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws/docs/10_desktop_application.md).

## 4. Run modes directly from ROS terminals (without Qt)

Close the Qt app first. Run **one backend at a time** on a ROS domain. Run this
setup in **every** terminal, including command/monitor terminals:

```bash
cd "$HOME/projects/dual-scara-chess/Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=211
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
unset ROS_STATIC_PEERS
export GEOMETRY="$PWD/src/chess_workcell_description/config/workcell_sim.yaml"
# Optional robot-only software experiment, instead of the seated layout:
# export GEOMETRY="$PWD/src/dual_chess_description/config/chess_geometry_adaptive_sim.yaml"
```

Each launch below occupies its terminal. Send its test commands from a second
terminal with the same setup. Stop with Ctrl+C before launching another mode.
Do not start another joint publisher, executor or automatic turn owner alongside
an existing backend. Reading topics while Qt runs is fine; competing motion
commands during a desktop match are not.

### A. Manual sliders + RViz

```bash
ros2 launch dual_chess_description manual.launch.py geometry_file:="$GEOMETRY"
```

This uses `joint_state_publisher_gui`; it is separate from Qt's own slider mode.
For the standalone enclosure/board model only:

```bash
ros2 launch chess_workcell_description display.launch.py
```

### B. MoveIt planning / mock-controller execution

Terminal A:

```bash
ros2 launch dual_chess_moveit_config demo.launch.py \
  geometry_file:="$GEOMETRY" publish_mock_readiness:=true
```

Terminal B, one command at a time:

```bash
ros2 run chess_motion_interface square_target --arm robot1 --square e4
ros2 run chess_motion_interface square_target --arm robot1 --square e4 --execute
ros2 run chess_motion_interface square_target --arm robot1 --home --execute
ros2 run chess_motion_interface square_target --arm robot2 --home --execute
# Only when a4 is empty: approach, descend, retreat; does not grip.
ros2 run chess_motion_interface square_target --arm robot1 --square a4 --probe --execute
```

With the new seated geometry, board commands return `unreachable_*`; home is
the validated controller test. In RViz select `robot1` or `robot2` in
MotionPlanning, set a position goal, Plan, then Execute. The SCARA has 3 DOF;
arbitrary 6D end-effector orientation is not supported. Reach/collision checks
still apply. RViz planning preview is not proof of controller execution.

### C. Gazebo physics + direct position control

Terminal A (Gazebo window):

```bash
ros2 launch dual_chess_gazebo gazebo.launch.py geometry_file:="$GEOMETRY"
```

For a server without a Gazebo window, use **instead**:

```bash
ros2 launch dual_chess_gazebo gazebo.launch.py \
  geometry_file:="$GEOMETRY" 'gz_args:=-r -s'
```

Terminal B, example joint commands and feedback:

```bash
ros2 topic pub --once /robot1/joint3/cmd_pos std_msgs/msg/Float64 '{data: 0.1}'
ros2 topic pub --once /robot2/joint1/cmd_pos std_msgs/msg/Float64 '{data: 0.01}'
ros2 topic echo /joint_states --once
ros2 topic echo /clock --once
```

The existing combined Gazebo/RViz bringup is also retained; use it **instead**
of the Gazebo-only launch, with extra game owners disabled:

```bash
ros2 launch dual_chess_bringup simulation.launch.py geometry_file:="$GEOMETRY" \
  chess_brain_enabled:=false auto_coordinator_enabled:=false executor_enabled:=false
```

Gazebo direct controls have no MoveIt trajectory-controller bridge or complete
dynamic chess-piece manipulation. Gazebo does not make unreachable board
pickups feasible. Profile PID values are simulation-only, not hardware tuning.

### D. Full deterministic mock chess pipeline

Terminal A:

```bash
ros2 launch dual_chess_moveit_config simulation_pipeline.launch.py geometry_file:="$GEOMETRY"
```

Terminal B:

```bash
ros2 run chess_motion_interface mock_engine --move e2e4
```

The command waits for execution and matching fresh vision, and exits nonzero on
rejection/failure. A successful command is required before the next move. The
seated profile rejects safely; this is not a setup error. `--no-wait` reports
command acceptance, not completed motion.

### E. Human/Stockfish backend

The **Qt mode is the normal automatic match entry point**. For backend inspection
without Qt, Terminal A can launch:

```bash
ros2 launch dual_scara_desktop stockfish_match.launch.py \
  geometry_file:="$GEOMETRY" stockfish_path:=/usr/games/stockfish
```

This standalone launch has **no automatic turn coordinator**. Terminal B may
submit a human UCI command, then request one engine move only after that move
has actually committed and the backend reports readiness:

```bash
ros2 topic pub --once /chess/command std_msgs/msg/String \
  '{data: "{\"action\":\"uci\",\"uci\":\"e2e4\",\"request_id\":\"manual-demo-1\"}"}'
# Only after confirmed success/readiness, not after a rejection:
ros2 topic pub --once /chess/command std_msgs/msg/String \
  '{data: "{\"action\":\"next\",\"request_id\":\"engine-demo-1\"}"}'
```

### F. Chess logic only / retained offline executor

```bash
ros2 launch dual_chess_bringup chess_logic_only.launch.py \
  auto_coordinator_enabled:=false executor_enabled:=false stockfish_path:=/usr/games/stockfish
```

This is not a self-contained robot game: external vision/safety sync is required.
The retained legacy executor is an offline prototype, not the current SCARA
MoveIt adapter. Its logging-only mode can be inspected in a separate session:

```bash
ros2 launch chess_robot_executor executor.launch.py dry_run:=true sink:=log
```

## 5. Special-move mock scenarios

Use **Full mock chess** in Qt for the simplest scenario selection. For CLI use,
stop the preceding session and launch exactly one of the following in Terminal A.
Run the indicated move command in Terminal B after mock vision/readiness sync.
On the seated profile each is expected to reject before activity, keeping FEN
unchanged. These commands do not assert that a physical multi-piece path exists.

**Exact previous failure after c4/e5** — Terminal A:

```bash
ros2 launch dual_chess_moveit_config simulation_pipeline.launch.py geometry_file:="$GEOMETRY" \
  'initial_fen:=rnbqkbnr/pppp1ppp/8/4p3/2P5/8/PP1PPPPP/RNBQKBNR w KQkq - 0 2'
```

Terminal B: `ros2 run chess_motion_interface mock_engine --move g1f3`

**Capture** — Terminal A:

```bash
ros2 launch dual_chess_moveit_config simulation_pipeline.launch.py geometry_file:="$GEOMETRY" \
  'initial_fen:=rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2'
```

Terminal B: `ros2 run chess_motion_interface mock_engine --move e4d5`

**Castling** — Terminal A:

```bash
ros2 launch dual_chess_moveit_config simulation_pipeline.launch.py geometry_file:="$GEOMETRY" \
  'initial_fen:=r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1'
```

Terminal B: `ros2 run chess_motion_interface mock_engine --move e1g1`

**En passant** — Terminal A:

```bash
ros2 launch dual_chess_moveit_config simulation_pipeline.launch.py geometry_file:="$GEOMETRY" \
  'initial_fen:=4k3/8/8/3pP3/8/8/8/4K3 w - d6 0 1'
```

Terminal B: `ros2 run chess_motion_interface mock_engine --move e5d6`

**Promotion** — Terminal A:

```bash
ros2 launch dual_chess_moveit_config simulation_pipeline.launch.py geometry_file:="$GEOMETRY" \
  'initial_fen:=4k3/P7/8/8/8/8/8/4K3 w - - 0 1'
```

Terminal B: `ros2 run chess_motion_interface mock_engine --move a7a8q`

If execution on a feasible simulation profile reaches `operator_required`, use
Qt's current-token confirmation button. For a CLI-only **synthetic operator**,
run `ros2 run chess_motion_interface mock_promotion_operator` in a third sourced
terminal before submitting promotion. This is not a real piece replacement.

## 6. Inspect state, inject mock failures, stop

Monitoring terminals use the same ROS setup/domain as their backend:

```bash
ros2 topic echo /chess/state --full-length
ros2 topic echo /chess/events --full-length
ros2 topic echo /chess/motion/status --full-length
ros2 topic echo /chess/robot_ack --full-length
ros2 topic echo /chess/geometry --once --full-length
```

Choose one topic per terminal or stop an echo with Ctrl+C before the next.
With a full mock/Stockfish backend, these existing controls are available:

```bash
ros2 param set /mock_gripper fail_next true
ros2 param set /mock_chess_vision confirm_after_ack false
# Later release held confirmation; a new mock observation is still required:
ros2 param set /mock_chess_vision confirm_after_ack true
# Software stop keeps the backend alive; false does not authorize unsafe resume:
ros2 topic pub --once /chess/motion/estop std_msgs/msg/Bool '{data: true}'
```

Preflight rejection may occur before a gripper/vision test can be exercised on
the seated profile. See the prior adaptive-profile evidence for execution waits.
For a synthetic board-pose input in an idle mock backend (not a real camera):

```bash
ros2 run chess_motion_interface mock_vision_input --position 0 0 0 --rpy 0 0 0
```

This pose is the board underside frame, not an arbitrary new elevation. Geometry
updates must be collision/reach checked again. Mount changes require relaunch.
A physical resize/pose/TCP change belongs in YAML/backend validation, not Qt pixels.

## 7. Tests and troubleshooting

From the sourced workspace:

```bash
export PYTHONPATH="$PWD/.qt-deps${PYTHONPATH:+:$PYTHONPATH}"
QT_QPA_PLATFORM=offscreen python3 -m pytest src -q
QT_QPA_PLATFORM=offscreen colcon test --return-code-on-test-failure \
  --test-result-base .desktop-state/teammate-test-results
colcon test-result --test-result-base .desktop-state/teammate-test-results --verbose
python3 tools/static_audit.py
```

Recorded project checks: **223 pytest tests**, **197 colcon tests**, 10 built
packages. These are recorded results on the development laptop, not a claim
that a fresh laptop install or a complete seated-layout chess game was tested.
See [current machine-readable evidence](Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws/docs/mount_validation/results.json).

| Symptom | Action |
|---|---|
| `Package ... not found` | Use the workspace path above, build, then source Jazzy and `install/setup.bash` in that terminal. |
| PySide6 missing | Run `scripts/install_desktop_dependencies.sh`; use `scripts/run_desktop.sh`. Recreate `.qt-deps` locally, not from somebody else's compiled environment. |
| Qt display/plugin failure | Run from a graphical desktop; install `libxcb-cursor0`. Unset `QT_QPA_PLATFORM` for normal GUI use; offscreen is only for tests. |
| Controllers or mode not ready | Inspect Qt logs/ROS status, confirm dependencies, and close competing backends on the same domain. |
| `unreachable_*` on new platform | Expected mechanical constraint. Read the mount report; do not change joint limits or move bases off seats to force a pass. |
| Motion completed but board pending | Fresh matching vision/operator confirmation is still required. Inspect status and avoid duplicate commands. |
| Stockfish not found | Install `stockfish`; confirm `/usr/games/stockfish` or set its actual path in Qt/launch. |
| Old installation/mounts appear | Open a clean terminal; source only Jazzy and this overlay. Verify the selected geometry file, then rebuild after source edits. |
| Updating this clone later | With local edits saved separately: `git pull --ff-only`, then `colcon build` and source the overlay. |

The saved evidence contains development-machine paths for provenance. They are
not required on another laptop. Runtime asset lookup uses installed ROS packages
and the chosen workspace/profile. No camera algorithms, embedded hardware,
microcontroller protocol or final mechanical mounting are assumed.

## More detail

- [Workspace README](Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws/README.md)
- [Qt architecture, controls and recovery](Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws/docs/10_desktop_application.md)
- [ROS motion, vision, chess-engine and embedded contracts](Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws/docs/09_configurable_motion_interfaces.md)
- [Measured mounts, mechanical limits and current test matrix](Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws/docs/14_mount_alignment.md)
- [Documentation index](Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws/docs/README.md)
