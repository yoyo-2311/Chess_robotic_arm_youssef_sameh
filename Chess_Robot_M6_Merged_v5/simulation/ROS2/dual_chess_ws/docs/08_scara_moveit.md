# SCARA integration and workspace analysis — 16 November 2026

## Backup and provenance

Baseline commit: `30a97e591bdd92fd49da5a5fbb5b46bda8aa25d1`.
[Original project backup branch](https://github.com/yoyo-2311/Chess_robotic_arm_youssef_sameh/tree/backup/original-before-scara-2026-11-16). This upload was verified with `git ls-remote` before source edits.

Input ZIP SHA-256: `1ef417bdf6c43dacc5011772e69f06d45266a419f1daa38b799820191d6e7a76`.
Only source URDF/Xacro, meshes and README were imported; ZIP build/install/log directories were excluded. Unmodified imported files are under `dual_chess_description/reference/scara_v2/`. Active mesh copies are byte-identical. Regression tests compare source origins, axes, limits and inertias with the integrated macro.

## What the folder contains

| Component | Finding and effect |
|---|---|
| `simulation/urdf_convert` | Older ROS 1 SolidWorks export; preserved as reference, outside the ROS 2 src directory. |
| `dual_chess_description` | Former arm had added base yaw and temporary wrist/gripper. Active top-level Xacro now instantiates supplied SCARA twice. Old macros/meshes remain as history. |
| `dual_chess_gazebo` | Gazebo Harmonic world and ROS bridge; bridge now has joint1/2/3 for each arm. Not the MoveIt mock backend. |
| `dual_chess_bringup` | Existing Gazebo/chess launch retained; executor remains off by default. |
| `chess_brain_handover` | Symbolic game state, Stockfish and move-plan interface; row/column reference CSV has no physical dimensions to rescale. |
| `chess_auto_coordinator` | High-level turn coordination, passive by default; does not perform collision-aware joint planning. |
| `chess_robot_executor` | Tested offline planar IK/trajectory prototype. Its arm lengths, joint convention, bins, gripper and board dimensions are the historical example, not this CAD. Live mode now rejects startup instead of publishing incompatible commands. |
| `dual_chess_moveit_config` | New SRDF, individual and combined groups, position-only KDL, OMPL, joint limits, mock ros2_control, two FollowJointTrajectory controllers, RViz, square planner and reach/path checker. |

Chess rules and robot motion remain separate. Full autonomous chess needs a new adapter from `/chess/move_plan` to MoveIt pick/place stages, gripper control and acknowledgements after verified motion; calling the single-square helper does not supply that integration.

## Supplied kinematics

| Joint | Type | Source limits |
|---|---|---|
| joint1 | vertical prismatic | 0–0.10 m; 0.10 m/s |
| joint2 | revolute | ±3.14159 rad; 1 rad/s |
| joint3 | revolute | ±3.14159 rad; 1 rad/s |
| joint4, joint5 | fixed gripper assembly | no opening command |

The source world-to-base rotation (`-pi/2` about X) is retained. Each arm adds a fixed mount at world Y=±0.19 m, Z=0.04 m and yaw ±pi/2. All source link inertias and internal joint transforms remain intact. No planar link-length approximation is used by the new helpers.

`tool0` is fixed to link5 at `(0.007, -0.0287, 0)` m. This is the geometric fingertip centre derived from link5 STL bounds, not a measured grasp/contact calibration. Link5 local Y points upwards in the assembled model, so its negative Y fingertip offset matters to board clearance. The old link5 origin alone falsely placed the fingers through the board at low targets.

The source URDF's effort/velocity values are accepted as supplied values, not measured motor capabilities. MoveIt acceleration limits (0.2 m/s² slide, 2 rad/s² rotary) and 20% speed/acceleration scaling are simulation choices. Physical commissioning needs measured limits and an appropriate ros2_control driver.

## Board choice and reach method

The original 400 mm frame/350 mm playing grid with bases at ±300 mm does not cover the complete board with the supplied arm. A numerical comparison with original link5 reference and heights 30/80 mm reached only 74/128 targets for one symmetric arm; that preliminary comparison is not the final fingertip validation.

The selected board scales XY by 0.6 while retaining the original mesh and vertical dimensions:

| Dimension | New value |
|---|---:|
| Outer frame | 240 mm |
| Playing grid | 210 mm |
| Square pitch | 26.25 mm |
| Rim width | 15 mm |
| Playing surface Z | 10 mm |
| Rim top Z | 20.1892 mm |
| Square-centre X/Y extrema | ±91.875 mm |
| Base spacing | 380 mm |
| CAD base-frame height | 40 mm |

This is a feasible simulated layout, not a proof of the largest possible board. Board visual scaling, collision boxes, checkerboard overlays and square TF frames all follow the same dimensions.

`check_square_reach.py` reads the expanded URDF, composes every fixed and actuated transform and solves bounded 3D numerical IK from six seeds. It tests both arms at every square centre and world Z=0.03/0.08 m. `--collision` asks MoveIt's whole-scene checker to validate candidate states with the other arm at SRDF home `(0.09, 0, 0)`. Only adjacent mechanical collision pairs are disabled in SRDF; inter-arm, board and non-adjacent self collisions remain enabled. The box approximation of the board includes its raised frame; the square overlays are visual only.

`--plan` additionally requests collision-aware OMPL paths from both arms parked. OMPL is stochastic. Up to five bounded planning requests are allowed per target, and a candidate is accepted only if MoveIt's final trajectory validator succeeds. The report records requests and final error codes. This does not prove every transition between arbitrary board poses or safe simultaneous motion.

## Planning details and limitations

Initial testing found that time parameterization could produce a trajectory rejected by the final collision validator. The final config uses finer OMPL collision sampling (0.001 longest valid segment fraction), TOTG path tolerance 0.00001, 0.02 s resampling and retains ValidateSolution after timing. This is significant for the mixed prismatic/revolute chain; MoveIt itself warns that TOTG path tolerance is imperfect for mixed joint types. Failed candidates are rejected and replanned, never executed. See [MoveIt's time parameterization documentation](https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/time_parameterization/time_parameterization_tutorial.rst) and [response-adapter implementation](https://moveit.picknik.ai/main/api/html/add__time__optimal__parameterization_8cpp_source.html).

The default launch uses [ros2_control mock components](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html) with real FollowJointTrajectory actions and state feedback, not physical dynamics. Existing Gazebo controllers remain a separate direct-topic simulation path, not connected to MoveIt. The source base mesh extends below the chosen board-relative world origin; floor clearance and supports require review before physics simulation. No Gazebo-dynamics success is claimed.

The 26.25 mm square pitch and wide supplied gripper need checking against the actual chess pieces and neighbouring pieces. No pieces are present in the planning scene. Tool orientation is underactuated: the planner controls XYZ, not arbitrary roll/pitch/yaw. Fingertip height 80 mm is near the model's vertical reach limit, and king clearance may be insufficient for a real set. Captured-piece storage is not validated. Vision and physical safety/stepper hardware are not integrated.

## Verification

- Baseline uploaded and remote hash verified before edits.
- All seven ROS packages built successfully on installed ROS 2 Jazzy/MoveIt.
- Expanded URDF passes `check_urdf`; no duplicate or missing tree elements.
- Static audit passes default, Gazebo, one-arm and no-arm Xacro variants.
- 132 tests pass across description, legacy executor, brain and coordinator, including source mechanics/mesh preservation.
- Each arm reaches all 64 squares at both tested heights with whole-scene endpoint collision checks (256/256 endpoints); idle arm parked.
- Real MoveIt ExecuteTrajectory action succeeds on mock controllers for robot1 to e4, home, robot2 to e5, home. A test with the other arm left on the board was rejected; no collision pairs were disabled to bypass it.
- All 256 final square paths pass MoveIt planning and final timed-trajectory collision validation, including bounded replanning where needed. Full results are recorded in `docs/scara_validation/` along with endpoint and mock execution evidence.

Commands are in the workspace README. Headless validation used a separate ROS domain and temporary build/install/log folders, so it did not overwrite another workspace installation.
