# New AssemBase CAD workcell: integration and validation

> Mount positions and validation in this historical report are superseded by [measured seat alignment](14_mount_alignment.md). The old off-platform success results do not validate the seated layout.


## What was supplied and preserved

`/home/youssef-sameh/Downloads/AssemBase.SLDASM.zip` is a ROS 1 SolidWorks export,
not a ROS 2 workspace. It supplies a control enclosure and fixed board, no arms,
mount transforms, embedded hardware, or camera. It has been integrated into the
existing checkout without replacing existing packages or vision. No commit/push.

`chess_workcell_description/reference/original` preserves the export. Provenance
and archive hash are recorded alongside it. The meshes are unchanged. The
normalized ROS 2 static environment uses valid package/link names and keeps CAD
assets separate from motion, game coordination, Qt and vision. Invalid exported
inertias (box COM outside the mesh) are retained only in reference, not used to
invent a physics model.

Measured mesh extents, in normalized world axes:

- Board: 280 × 280 × 10 mm; bottom Z=0, top Z=10 mm.
- Enclosure: X ±625 mm, Y ±225 mm, Z=-231.585 to +10 mm.
- Native -Y up is converted with Rx(-pi/2). The original box-to-board fixed joint
  and normalized poses are checked numerically in unit tests.

The enclosure is fixed independently of the board pose. Existing board/vision
pose updates move the board and pieces, not the enclosure or mounts. In MoveIt,
only the scene owns CAD collisions. In manual/Gazebo URDF visualization, only the
URDF owns them. A regression test prevents duplicate geometry ownership.

## Simulation profile, not final mounting measurements

Use `src/chess_workcell_description/config/workcell_sim.yaml`. This explicitly
provisional layout retains a 26.25 mm playing pitch with 35 mm borders on the
280 mm slab. The CAD does not specify the actual playing grid. Arm mount origins
are [0,-0.27,0.04] and [0,0.27,0.04] m, yaws pi and zero; columns face outward.
Home positions are [0.09,-pi/2,0] (metres, radians, radians). These placeholders
must be replaced by measured mounting geometry before physical use. The imported
base mesh reaches about Z=-0.398 m at this mount height, below the enclosure
bottom (-0.232 m); an actual outboard support/floor arrangement has not been
specified or validated. A collision-free mock scene is not a mounting design.

Earlier layouts were tested and rejected, not hidden by collision exclusions:

- At Y ±0.21 m with the original inward-facing bases, both base meshes intersected
  the new enclosure (`initial_scene_collision`).
- With outward-facing bases but still Y ±0.21 m, c2 approach folded link3 into
  link1 (`collision_at_c2_approach`), before movement.
- Moving the columns outward in the separate simulation profile resolved those
  tested conflicts. Robot meshes, joint limits and collision checks are intact.

The pure IK audit checks pawn/knight grasps and highest approach at all 64 squares
for each arm: 384 targets, zero IK failures. This is **not** collision/path proof
for every square. Full trajectories must still pass MoveIt with the current
pieces, payload, other arm and enclosure present.

## Pickup and placement planning

The profile opts into a bounded set of grasp fractions [.65,.8,.9,.98,1.0]. These
are simulated grasps, not calibrated gripping contact. Before any actuation,
the backend searches only these approved fractions and checks both pickup and
placement descents, including the carried piece geometry. Travel height tracks
the selected grasp so the payload retains configured clearance. No failed robot
motion is automatically retried. Profiles without this option retain their one
configured grasp point.

The temporary payload preview restores both the source world object and its
allowed collision matrix even when planning fails. Without restoring the matrix,
MoveIt's world-object remove/re-add can erase selected-piece touch permissions.
Restoration uncertainty is blocking; it is never advertised as a safe rejection.
All actual descents are collision-checked again before controller execution.

Only Chess Brain plus fresh matching vision commits a move. A proof that a
rejection occurred before any arm/gripper activity allows another human move
after ROS clears the pending plan and reports fresh readiness. Partial or
uncertain execution retains the existing recovery flow. No vision algorithms,
message schemas or confirmation rules were replaced.

## Build and run

From `simulation/ROS2/dual_chess_ws`:

```bash
source /opt/ros/jazzy/setup.bash
# Dependencies and desktop setup: scripts/install_desktop_dependencies.sh
colcon build
source install/setup.bash
ros2 run dual_scara_desktop desktop --workspace "$PWD" --geometry-file "$PWD/src/chess_workcell_description/config/workcell_sim.yaml"
```

Select Human vs Stockfish, White, then Start. The geometry file picker remains
available. The old default profile is preserved so existing launches do not
silently acquire provisional CAD mounting changes. The above command explicitly
selects the new assembly. Manual mode initializes its sole joint publisher and
sliders from the same configured home as ros2_control; feedback supersedes it.

Standalone assembly view:

```bash
ros2 launch chess_workcell_description display.launch.py
```

Repeat the exact c4/e5 setup and Qt-board tests:

```bash
export PYTHONPATH="$PWD/.qt-deps:$PYTHONPATH" # only for workspace-local PySide6
QT_QPA_PLATFORM=offscreen python3 src/dual_scara_desktop/tests/gui_exact_position_probe.py --workspace "$PWD" --domain 224 --moves g1f3 g1h3 b1c3 e2e4 --geometry "$PWD/src/chess_workcell_description/config/workcell_sim.yaml" --tag workcell_repeat
```

Each move starts a fresh game, physically executes c4/e5 using deterministic
commands, then clicks the Qt board and verifies exactly one Stockfish reply,
fresh-vision commits, White's return, h2 present and clean owned-process shutdown.
Add `--hold-vision` or `--fail-gripper` for failure cases. `--transport ros` bypasses
board clicks and disables the desktop automatic turn owner for that test.

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest src -q
colcon test
colcon test-result --verbose
python3 tools/static_audit.py
python3 tools/workcell_reachability.py --geometry src/chess_workcell_description/config/workcell_sim.yaml --output docs/workcell_validation/reachability.json
```

## Limits and revalidation

This is MoveIt with mock ros2_control, mock gripper and existing mock vision.
It does not certify a physical robot, real finger grasp, camera or embedded
controller. New-enclosure Gazebo dynamics have not been validated; the CAD's
inertias are unsuitable as supplied. Manual mode is visualization only.

After changing dimensions, TCP, mounts, board pose, piece heights or gripper:
validate CAD extents/transforms; check initial whole-scene collisions; rerun all
square IK; plan both elbow branches and full pickup/placement descents; test
payload transfer/retreat, other-arm parking and capture bins; then replay fresh
confirmed games and failure/vision tests. Keep physical measurements in geometry
configuration and hardware plugins, not Qt widgets or chess rules.

## Recorded checks

See `docs/workcell_validation/results.json` for the final check list and
`docs/pickup_validation/new_cad_*` for Qt reports, event traces and screenshots.

## Files changed for this increment

- New `src/chess_workcell_description/`: CAD originals/provenance, normalized
  URDF/Xacro, immutable meshes, geometry profile, adapter, launch and asset tests.
- `dual_chess_description/package.xml` and `urdf/{dual_chess.urdf.xacro,
  board/board.macro.xacro,arm/scara_v2.macro.xacro}`: optional workcell and
  configured mock home initialization; old profiles retain prior defaults.
- `dual_chess_moveit_config/launch/demo.launch.py`: profile-aware named home.
- `chess_motion_interface/{package.xml,chess_motion_interface/{geometry.py,
  backend.py,node.py,scene.py},tests/{test_geometry_operations.py,
  test_descent_preflight.py}}`: CAD scene adapter, home configuration, bounded
  pickup/placement preflight and collision-permission restoration regressions.
- `dual_scara_desktop/dual_scara_desktop/{app.py,controller.py,ros_bridge.py}`:
  explicit geometry CLI and shared manual initial home; existing picker remains.
- `dual_scara_desktop/tests/{native_probe.py,gui_exact_position_probe.py,
  test_qt_controls.py,test_supervisor.py}`: geometry-aware probes, immediate
  setup-rejection diagnosis, home-display regression and isolated supervisor
  test lock identifiers so live simulation domains cannot invalidate unit tests.
- `tools/workcell_reachability.py`, workspace `README.md`, this document and
  validation evidence. Existing accumulated project changes remain untouched.


Final checks on 2026-11-23: 10 packages built; 217 pytest tests passed; colcon
reported 185 tests with zero errors/failures/skips; static audit and check_urdf
passed. All four Qt moves after c4/e5 passed with exactly one confirmed reply:
g1f3→b8c6, g1h3→g7g6, b1c3→f8b4, e2e4→g8f6. Each returned White's turn
and stopped owned processes cleanly. Delayed vision kept FEN unchanged and
blocked progression until released, then completed. Injected gripper failure
showed a visible error, preserved the two-ply confirmed FEN and requested no
reply. Native Wayland manual and MoveIt/RViz readiness/software-stop/clean-close
checks passed. The legacy adaptive profile also completed Nf3/Nc6, selecting
0.9-height grasps where the old 0.65 point collided; the CAD layout passed these
moves at its preferred 0.65. These are focused execution tests, not certification
that every legal move or a complete physical-motion game is feasible.
