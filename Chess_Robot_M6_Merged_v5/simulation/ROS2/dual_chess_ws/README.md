# Dual SCARA chess robot — ROS 2 Jazzy / MoveIt 2

This continues the supplied `Scara_ws_last.zip` integration. Both CAD arms, previous launch names, chess brain, coordinator and the offline executor are retained. The new `chess_motion_interface` connects the existing chess action schema to MoveIt and controller feedback.

**Start here:** [complete test guide and engine/vision/embedded contracts](docs/09_configurable_motion_interfaces.md).

## Build

From this `dual_chess_ws` folder:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Source both environments in every terminal. Run one test mode per ROS domain.

## Test modes

| Mode | Command |
|---|---|
| Manual sliders + RViz | `ros2 launch dual_chess_description manual.launch.py` |
| Interactive MoveIt + mock controllers | `ros2 launch dual_chess_moveit_config demo.launch.py` |
| Full mock engine/vision/gripper → robot → feedback | `ros2 launch dual_chess_moveit_config simulation_pipeline.launch.py` |
| Existing display / Gazebo / chess-only launches | Retained; see the test guide for backend distinctions |

In another sourced terminal after starting the appropriate mode:

```bash
# With demo.launch.py: plan only, then optionally execute on mock controllers
ros2 run chess_motion_interface square_target --arm robot1 --square e4
ros2 run chess_motion_interface square_target --arm robot1 --square e4 --execute
ros2 run chess_motion_interface square_target --arm robot1 --home --execute
# With simulation_pipeline.launch.py: wait for legal move, execution and fresh mock vision
ros2 run chess_motion_interface mock_engine --move e2e4
ros2 run chess_motion_interface mock_engine --move d7d5
ros2 run chess_motion_interface mock_engine --move e4d5
```

The guide includes separate capture, castling, en passant, promotion, mock-CV, stop, failure and confirmation tests. `square_target --probe --execute` performs a straight descent/retreat on an **empty** square. The original `plan_square.py` and all-square checker now read live geometry.

## Configurable, low-mounted board

Edit `src/dual_chess_description/config/chess_geometry.yaml`, or pass `geometry_file:=/absolute/path/custom.yaml` to a launch. It centralizes board dimensions, rectangular square pitches, XYZ/RPY, separate robot mounts, support, piece heights, clearances, TCP and capture trays. Board settings also have live ROS parameters on `/chess_motion_interface`; active motion and pending vision confirmation lock geometry changes. Every square and scene object follows the effective geometry.

**Every physical value is a simulation placeholder.** Initially the playing surface is just 10 mm above its supporting pad; CAD base origins are separately at 40 mm. The final board, platform, TCP, mounting geometry and chess set are unconfirmed. The supplied CAD base intersects a broad platform at this low level, so a successful test with the configured small pad is not proof that an arbitrary tabletop will work.

## Execution and integration boundaries

MoveIt uses position-only IK for each 3-DoF arm. XYZ goals are supported; arbitrary tool orientation is not. Execute sends actual ROS trajectory actions to **mock ros2_control**, and success also requires endpoint joint feedback. This does not simulate Gazebo contact physics or real motors.

The supplied gripper is fixed. The new pipeline uses an explicit mock gripper handshake and attached collision objects to test pick/place logic. A future hardware adapter must supply real gripper actuation/feedback and a ros2_control driver after the embedded specifications are known. No MCU, motor or protocol is assumed.

The existing chess brain retains rule validation and confirmed game state. Robot success alone never commits a move: a fresh matching vision observation is also required. Failed or partial movement cannot advance confirmed FEN. Mock vision is synthetic test evidence and must not be used to confirm a physical board.

## Evidence and history

Current reports: `docs/extension_validation/`. Current architecture/commands: [test guide](docs/09_configurable_motion_interfaces.md).

[Earlier SCARA integration](docs/08_scara_moveit.md) and [pre-SCARA README](docs/reference_pre_scara_README.md) are historical. The unchanged original was uploaded before edits as commit `30a97e591bdd92fd49da5a5fbb5b46bda8aa25d1` on [the backup branch](https://github.com/yoyo-2311/Chess_robotic_arm_youssef_sameh/tree/backup/original-before-scara-2026-11-16).
