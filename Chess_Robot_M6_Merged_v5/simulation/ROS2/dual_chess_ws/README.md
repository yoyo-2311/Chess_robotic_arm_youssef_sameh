# Dual chess SCARA robot — ROS 2 Jazzy and MoveIt 2

The active robot description now uses **Scara_ws_last.zip**, instantiated twice with unique joint/link names. Original CAD transforms, limits, masses, inertias and six STL meshes are retained. The previous arm files remain for reference but are not included by the active model.

The board is **240 mm outside, 210 mm playing area, 26.25 mm squares**. Bases are at world Y=±190 mm, Z=40 mm, yaw ±90°. Tool frames use the geometric fingertip centre. This layout is for simulation and needs physical calibration.

The unchanged project was committed as `30a97e591bdd92fd49da5a5fbb5b46bda8aa25d1` and [uploaded before modification](https://github.com/yoyo-2311/Chess_robotic_arm_youssef_sameh/tree/backup/original-before-scara-2026-11-16).

## Build and launch MoveIt

Run these commands from this `dual_chess_ws` directory:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch dual_chess_moveit_config demo.launch.py
```

Use `rviz:=false` for headless operation. This launch runs **mock ros2_control hardware**, two trajectory controllers, robot_state_publisher and MoveIt. It does not start Gazebo or command motors. Avoid running the old display/Gazebo launch on the same ROS domain at the same time: each publishes its own robot/joint state.

In another sourced terminal:

```bash
# Plan only; position goal at fingertip height 80 mm in world coordinates
ros2 run dual_chess_moveit_config plan_square.py --arm robot1 --square e4
# Plan and execute on the mock controllers
ros2 run dual_chess_moveit_config plan_square.py --arm robot1 --square e4 --execute
# Park before switching arms
ros2 run dual_chess_moveit_config plan_square.py --arm robot1 --home --execute
ros2 run dual_chess_moveit_config plan_square.py --arm robot2 --square e5 --execute
ros2 run dual_chess_moveit_config plan_square.py --arm robot2 --home --execute
# Full endpoint + path check, with both arms initially parked; does not execute paths
ros2 run dual_chess_moveit_config check_square_reach.py --plan --output /tmp/chess_square_paths.json
```

In RViz choose planning group `robot1`, `robot2` or `both_arms`. Only the individual arms have position-only IK; use joint goals for `both_arms`. A 3-DoF SCARA cannot choose arbitrary tool orientation. The square helper derives joint goals from the actual CAD chain and calls MoveIt's collision-aware planner, then optionally ExecuteTrajectory.

## Scope and limits

The empty-board checks cover every square centre at fingertip world heights 30 and 80 mm, with the other arm parked. These are position/path checks, not a complete chess pick-and-place demonstration. Gripper joints in the supplied export are **fixed**. A calibrated grasp point, actuated gripper, piece geometry/clearance, capture bins, physical drivers and chess-move orchestration remain to be integrated. An 80 mm tool height is not sufficient clearance for every possible physical chess set.

The original `chess_robot_executor` is retained as an **offline planar prototype**, with its original example dimensions and tests. Its live mode is disabled because it does not describe this CAD chain. The chess brain and coordinator remain available, but do not route chess plans into the new MoveIt helper yet.

The existing Gazebo launch expands the new model and uses six direct position-command channels. It is separate from the tested mock MoveIt system. Gazebo dynamics, CAD floor clearance and physics controllers have not been validated in this change. Do not treat mock execution as Gazebo or hardware verification.

See [analysis, geometry and verification](docs/08_scara_moveit.md). The [previous README](docs/reference_pre_scara_README.md) describes the preserved baseline and is historical.
