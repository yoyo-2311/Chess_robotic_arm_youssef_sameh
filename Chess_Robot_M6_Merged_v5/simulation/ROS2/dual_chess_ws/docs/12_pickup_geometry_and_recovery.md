# Pickup rejection after `1. c4 e5`: diagnosis and recovery

The screenshot was reproduced using the unchanged default 0.24 × 0.24 m board,
0.02625 m pitch, Human vs Stockfish, White/robot1. Each test physically executed
and vision-confirmed `c2c4` and `e7e5` through the existing mock motion pipeline;
the desktop's automatic coordinator was disabled only during those two
deterministic setup commands, then restored for the human move. No FEN was
injected into Stockfish, no pawn was removed, and mock vision was not replaced.

The setup FEN was:

```text
rnbqkbnr/pppp1ppp/8/4p3/2P5/8/PP1PPPPP/RNBQKBNR w KQkq - 0 2
```

**Result:** the default configured grasp is infeasible, rather than the high
approach merely choosing the wrong usable branch. A separately named
simulation profile with a higher grasp point passes the knight moves. The
default remains unchanged. Proven pre-activity rejections now clear the pending
request through Chess Brain and return the human controls after fresh readiness;
partial or uncertain executions remain blocked.

## Which branch and which point collided?

All positions below are numerical measurements in the simulation model, in
metres in `world`. They are not measured hardware calibration values.

The default `g1` approach TCP is `[0.065625, -0.091875, 0.0623]`; its grasp TCP
is `[0.065625, -0.091875, 0.0243]`.

| Candidate | Approach joints `[prismatic m, rotary rad, rotary rad]` | Actual finding |
|---|---|---|
| 0 | `[0.079797922, 0.043917022, 2.859010751]` | High approach valid. Cartesian fraction `0.8947368421`. Last returned valid TCP Z: `0.028300`. Fine same-branch descent sweep: valid at Z `0.02802549`, first invalid at `0.02777712`. Contact: **robot1_link4 / chess_piece_h2**. |
| 1 | `[0.079798341, 2.737847707, -2.429723762]` | Already invalid at the high approach, before descent. Fraction `0.0`. Contact: **robot1_link3 / robot1_link1**. |

At candidate 0's first invalid sample, joints were approximately
`[0.045275046, 0.043917022, 2.859010751]`. The contact point was
`[0.091561129, -0.066955407, 0.027777270]`, with penetration depth
`0.000222730` m. The adjacent pawn cylinder's actual scene centre was
`[0.091875, -0.065625, 0.019]`, height `0.018`, radius `0.003`, so its top is
Z `0.028`. The contacting link4 surface passes down through that top as the TCP
descends below approximately 28 mm. The requested grasp is another 3.7 mm below
that level. Increasing only the **high** approach height cannot fix this
collision at the low endpoint.

At candidate 1's initial point, the self-contact was at
`[0.070595656, -0.153718470, 0.142049563]`, depth `0.000718471` m. Both candidates
are within joint limits. This branch is unusable regardless of its descent.

Evidence:

- [Original candidate paths and 0.25 mm validity sweep](pickup_validation/baseline2/qt_g1f3_inspection.json).
- [Actual scene object poses, repeated sweep and joint states](pickup_validation/default_recovery/qt_g1f3_inspection.json).
- [Mesh/contact audit](pickup_validation/default_recovery/mesh_contact_audit.json) and [source transform audit](pickup_validation/source_transform_audit.json).

The installed meshes equal the source meshes and the preserved archive meshes.
Visual/collision mesh geometry and origins match. Joint 1–5 origins, axes and
types match the archived source. Transforming the world contact into link4's
mesh frame lands on an actual STL triangle to numerical precision; the
self-contact similarly lies on link3's mesh, with the returned depth matching
penetration into link1. No inflated primitive, mesh-unit mismatch or inconsistent
visual/collision transform was found. This establishes consistency of the
deployed model; it does **not** establish that the unmeasured TCP/grasp settings
are correct for physical hardware.

The previous diagnostic attached contacts from **all** endpoint IK solutions to
each candidate. That could misleadingly associate the self-collision with the
otherwise valid high-approach branch. Diagnostics now report each candidate's
own start and nearest same-branch endpoint contacts separately. The aggregate
error still lists both actual obstacles.

## Separate simulation profile

New file:

```text
src/dual_chess_description/config/experiments/chess_geometry_upper_grasp_sim.yaml
```

It changes only `motion.grasp_fraction` from **0.65 to 0.90**. Board dimensions,
pitch, board height, arm mounts, TCP, piece dimensions, velocity/acceleration,
Cartesian step, jump thresholds and collision requirements remain unchanged.
See [machine-checked profile difference](pickup_validation/profile_diff.json).

For the knight, the requested grasp is now Z **0.0298** instead of `0.0243`:
1.8 mm above the neighbouring pawn's top and 2.2 mm below the knight's top. The
existing geometry formula also derives a higher approach, Z **0.0678**. The
board is not raised. A fresh sweep returns fraction **1.0** for candidate 0,
while candidate 1 still self-collides and is not used. The actual descent, lift,
transfer, placement and retreat completed, retaining `h2`.

See [live profile sweep](pickup_validation/upper_grasp_ros/ros_g1f3_inspection.json).
The general planner already restricts approach goals to branches with complete
collision-checked descents, then rechecks each actual descent from controller
feedback. No partial-path fallback or extra allowed collision pair was added.

**This profile is an experiment, not a promoted default or hardware approval.**
The mock gripper does not validate real finger closure, holding force or grasp
stability near a piece's top. Validate the actual TCP, finger geometry and chess
set before adopting a physical grasp. These successful cases do not prove
clearance for every square, piece arrangement or complete game.

## Safe rejection and Paused behaviour

The old failed acknowledgement blocked Chess Brain even when nothing had been
dispatched. The new path is deliberately narrower than a generic failure reset:

1. The motion worker counts arm-trajectory dispatch attempts and **all gripper
   command intents**. A preflight-specific exception is eligible only if both
   counters remain unchanged across the entire chess plan, no partial/unknown
   execution is recorded, and the selected-piece touch permission was restored
   successfully (or the rejection came from a read-only preflight).
2. That worker sends an explicit failed acknowledgement with `safe_rejection`,
   the matching `before_fen`, `arm_activity:false`, `gripper_activity:false` and
   `scene_restored:true`. Legacy failures keep their old semantics.
3. Chess Brain validates the matching pending plan/FEN and proof, refuses to
   downgrade a previous successful acknowledgement or blocked execution, and
   clears **only** the unexecuted pending plan. Confirmed FEN, history and turn
   do not advance. Its state reports `last_rejection`, so losing one transient
   event cannot leave the GUI dependent on that event alone.
4. Qt releases the matching in-flight guard only after that authoritative state
   says the pending request is cleared. It then waits for fresh motion-idle and
   safety-readiness messages after the release. The specific cause stays visible
   and the human can select another legal move. Duplicate/late rejection events
   cannot clear a newer request or pause the completed recovery again.

No automatic retry occurs. If a Stockfish move is safely rejected, the engine's
per-ply dispatch guard remains set and the exact cause remains visible; it does
not request alternative engine moves in a loop. Software stop, transport
uncertainty, failed gripper feedback, partial captures and any earlier activity
in a multi-action move still require the existing recovery flow. A successful
robot acknowledgement still cannot commit a move without fresh vision.

This is a proof about the app-owned simulated backend's activity and confirmed
state, not a substitute for a physical hardware safety interlock.

## Scenarios actually run

Each target case used a separate confirmed `c4 e5` session unless it explicitly
tested choosing another move after rejection.

| Profile / input | Result |
|---|---|
| Default, Qt `g1f3` baseline | Exact rejection reproduced; candidate sweep and contacts above. |
| Default, ROS `g1f3` after fix | Proven pre-activity rejection; pending cleared, board stays at two plies, fresh readiness restored. |
| Default, Qt `g1h3` after fix | Same source-grasp collision; controls restored for another human choice. |
| Default, Qt `b1c3` control | Human and one Stockfish `f8e7` reply confirmed; human turn returned. |
| Default, reject `g1f3`, then choose `e2e4` or `b1c3` in the same session | Human controls restored and the chosen move executed/committed. Exactly one Stockfish reply was requested; its `b8c6` pickup hit the mirrored `a7`/self-collision and was safely rejected. This is **not** reported as a completed reply. |
| Experimental, Qt `g1f3` | Human and one `b8c6` reply confirmed; human turn returned. |
| Experimental, Qt `g1h3` | Human and one `g7g6` reply confirmed; human turn returned. |
| Experimental, Qt `b1c3` | Human and one `f8e7` reply confirmed; human turn returned. |
| Experimental, ROS `g1f3` | Fraction-1 branch verified live; human and one `e5e4` reply confirmed; `h2` retained. |
| Experimental, Qt `g1f3`, vision held | Robot finishes but board/history remain at `c4 e5` and no reply is requested. After fresh vision is released, human and one `f7f6` reply commit. |
| Default, inject gripper failure on `b1c3` | Pending remains blocked, confirmed FEN remains `c4 e5`, no reply, explicit recovery required. It is not misclassified as a safe rejection. |

Raw outcomes: [JSON](pickup_validation/case_summary.json) /
[CSV](pickup_validation/case_summary.csv). Each case includes request/plan IDs,
timestamped commands, motion events, acknowledgements, authoritative state and
screenshots. The [confirmation audit](pickup_validation/confirmation_audit.json) checks four robot acknowledgements/fresh-vision commits (including setup) and exactly one accepted Stockfish reply command in every successful session. `complete:false` plus `safe_rejection_recovered:true` means the
configured target was rejected safely, not executed. `EXPECTED_SAFE_REJECTION`
is the probe's collection marker for that expected negative scenario.

All selected runs closed their app-owned workers/processes cleanly; measured Qt
timer gaps stayed below 0.1 seconds. Failed preliminary setup attempts are also
retained, but excluded from the pass matrix: two sessions timed out querying
state validity before target dispatch, and an early test-only coordinator/UI
readiness race caused no command. The probe was corrected and those cases were
rerun. State-validity queries now explicitly await service discovery before
dispatch. The live collision reproduction and successful profile validations
were not blocked by missing dependencies.

## Build, use and repeat

Close the current desktop application before rebuilding so its Python modules
reload; stopping only the ROS mode does not reload an already running Qt app.
From the workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
./scripts/run_desktop.sh
```

For the validated simulation experiment, use **Browse** before Start and select
`src/dual_chess_description/config/experiments/chess_geometry_upper_grasp_sim.yaml`.
Choose Human vs Stockfish / White. The normal default still rejects the low
`g1` grasp safely; selecting the experimental profile is explicit.

To reproduce the deterministic exact-position tests:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/.qt-deps:$PYTHONPATH"
export QT_QPA_PLATFORM=offscreen

# Default: reject safely, then test another legal move without restarting.
python3 src/dual_scara_desktop/tests/gui_exact_position_probe.py \
  --workspace "$PWD" --domain 221 --moves g1f3 --expect-rejection \
  --recover-move b1c3 --inspect --tag repeat_recovery

# Separate profile: three fresh c4/e5 sessions with actual Qt board input.
python3 src/dual_scara_desktop/tests/gui_exact_position_probe.py \
  --workspace "$PWD" --domain 228 --moves g1f3 g1h3 b1c3 \
  --geometry "$PWD/src/dual_chess_description/config/experiments/chess_geometry_upper_grasp_sim.yaml" \
  --tag repeat_upper_grasp

python3 -m pytest -q src
colcon test --python-testing pytest
colcon test-result --verbose
python3 tools/static_audit.py
```

Use free ROS domains. The direct-ROS option is `--transport ros`; it disables the
desktop auto-coordinator while the driver owns the two command dispatches, so
there is exactly one turn owner. `--inspect` runs the read-only candidate sweep
with scoped selected-piece touch permissions restored afterward.

Final checks: **205 pytest tests passed**, **179 colcon tests passed**, **9
packages built**, and the static audit passed. Evidence is in
[pytest.txt](pickup_validation/pytest.txt),
[colcon_results.txt](pickup_validation/colcon_results.txt),
[build.txt](pickup_validation/build.txt) and
[static_audit.txt](pickup_validation/static_audit.txt).

## Files changed in this extension

- Motion backend/node: branch-specific diagnostics, discovery guard,
  preflight-specific no-activity proof and gripper-intent tracking.
- Chess Brain `core.py` / `ros_node.py`: validated rejection-only pending release;
  normal success/vision and uncertain-failure semantics retained.
- Desktop `coordinator.py` / `controller.py`: authoritative recovery, fresh
  readiness gating, stale-event protection and retained cause.
- Experimental YAML above; the default profile and URDF/mesh files were not changed.
- Regression tests in `test_safe_rejection.py`, `test_descent_preflight.py`,
  `test_coordinator.py`, `test_qt_controls.py`; new
  `gui_exact_position_probe.py`, `tools/inspect_pickup_scene.py` and
  `tools/audit_pickup_geometry.py`.
- This report and `pickup_validation/` evidence; a link in the desktop guide.

No camera, calibration, vision-node code, hardware driver or unrelated working
component was replaced. No commit or push was performed.
