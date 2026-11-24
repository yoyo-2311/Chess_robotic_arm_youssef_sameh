# Opening-move stall diagnosis and fix

The kingside-knight issue was reproduced through both Qt clicks and direct ROS
commands. It was **a collision-infeasible pickup descent on White/robot1**, not
a frozen Qt window or incorrectly serialized move. The configured `g1` grasp
cannot be executed in either tested IK branch with the starting board intact.
The fix rejects it **before arm movement**, reports the collision explicitly,
and checks complete descents when choosing approach branches for every piece.
It does not make the colliding knight moves physically executable by weakening
checks or changing measurements.

All changes remain local; no commit/push. Earlier uncommitted desktop and
full-game work was preserved. This document concerns the default profile in
this checkout, not an unobserved hardware configuration.

## Reproduction results

Every row used a fresh standard starting position, human White, robot1 White /
robot2 Black, the same geometry, mock ros2_control, existing mock gripper and
existing mock vision. Qt ran offscreen with real QtTest board clicks and a live
rclpy worker. These are MoveIt-controller simulation tests, not Gazebo contact
or hardware tests.

| Opening | Before: Qt | Before: direct ROS | After: Qt and direct ROS |
|---|---|---|---|
| `g1f3` | Accepted; high approach executes; pickup descent fails at fraction `0.8947`; White remains uncommitted | Same failure | Explicit pickup-preflight rejection; **zero measured joint travel**, zero committed plies, zero Stockfish replies |
| `g1h3` | Same pickup failure from `g1`, regardless of destination | Same failure | Same explicit pre-motion rejection and unchanged board |
| `b1c3` | Human confirmed, one Black `e7e5` reply confirmed | Human confirmed, one `e7e5` reply confirmed | Human and exactly one reply confirmed; replies were `g7g6` (Qt) / `d7d5` (ROS); human turn restored |
| `e2e4` | Human confirmed, one `e7e6` reply confirmed | Human confirmed, one `g8f6` reply confirmed | Human and exactly one reply confirmed; `e7e6` (Qt) / `g8f6` (ROS); human turn restored |

Stockfish is strength-limited and not deterministic; reply variation is
expected. No Qt freeze was observed. Timer-gap measurements and per-case
outcomes are in the [matrix CSV](stall_validation/opening_matrix.csv) and
[JSON](stall_validation/opening_matrix.json). The reproducible fresh-game
knight failure was consistent in these runs; the user's broader intermittent
symptom was not independently reproduced as a GUI freeze.

For the direct-ROS comparison, the same `{"action":"uci","uci":...,"request_id":...}`
is published through the existing rclpy `/chess/command` publisher without board
selection or GUI UCI validation. The desktop's automatic coordinator is disabled
in those cases; the test driver alone requests one `next` after authoritative
human confirmation and fresh readiness. In Qt cases, only the desktop owns turn
progression. No competing coordinator was started. This is not an external CLI
publisher that would intentionally trip the desktop's competing-owner guard.

## First failing stage and evidence

The baseline Qt `g1f3` trace correlates:

- Request `35c013e0989b42c8a4486a4d4fd30678` → accepted plan
  `2b6eb40123b94130b19781fac9e0e2db` at about 5.85 seconds.
- Chess Brain validates `g1f3` / `Nf3`, White, source `g1`, destination `f3`.
- Motion acceptance/preflight is followed by `approach_pick` at 6.34 seconds.
  The approach's controller execution completes, then the Cartesian descent
  fails before gripper close at 14.49 seconds.
- `/chess/motion/status` reports `cartesian_path_incomplete: fraction=0.8947 code=1`.
  `/chess/robot_ack` for that same plan is `success:false`.
- `/chess/events` reports `robot_ack_response` / blocked.
  `/chess/state` keeps the initial FEN, zero committed plies and pending `g1f3`.
  No fresh-vision commit or Stockfish `next` occurs.

See the [full timestamped baseline trace](stall_validation/before/qt_g1f3_events.json).
Each case includes `/chess/events`, `/chess/state`, `/chess/motion/status`,
`/chess/robot_ack`, plans and effective geometry; timestamps are monotonic seconds
relative to that run, with plan creation's Unix timestamp retained. Qt source /
destination clicks are recorded too. Subsequent traces also record the exact
published command JSON.

The live diagnostic queried `GetStateValidity` in the active collision scene,
with only the selected knight's existing gripper-touch permissions enabled.
The configured world grasp is `[0.065625, -0.091875, 0.0243]` metres. Both found
IK solutions are inside the model joint limits, but neither is collision-free:

| Grasp joints (prismatic m, rotary rad) | Returned collision |
|---|---|
| `[0.041798, 0.043917, 2.859011]` | `robot1_link4 / chess_piece_h2` |
| `[0.041798, 2.737848, -2.429724]` | `robot1_link3 / robot1_link1` |

The first branch hits the adjacent `h2` pawn; the alternative self-collides.
This is not a Z-limit violation or chess-legality failure. The supplied arm has
position-only IK, not arbitrary independently selectable tool orientation.
See [live contact-query evidence](stall_validation/diagnostic2/qt_g1f3_events.json).
The profile/model/limit hashes are recorded in
[geometry_and_limits.sha256](stall_validation/geometry_and_limits.sha256);
those files were not modified.

## Root causes and implementation

1. **Incomplete motion preflight.** Previously `check_targets` checked grasp IK
   reachability, but collision-checked only the high approach. `backend.move`
   could choose an approach branch that could not descend. The rejection was
   discovered only after approach execution. The new shared `Backend.approach`
   probes complete Cartesian descents from each candidate approach branch in the
   live collision scene, including the other arm and attached payload. Only
   feasible branches become approach goals. Pickup is checked before operating
   the gripper or dispatching an arm trajectory. On preflight rejection the
   selected-piece touch permission is restored. Placement uses the same branch
   check, and every actual descent is still replanned from measured feedback.
   Synthetic candidate states and partial paths are never executed. No collision
   pair was broadly disabled, and the configured fraction remains `1.0`.
2. **Specific failure hidden by generic state.** `MatchCoordinator.observe_state`
   replaced the detailed motion failure with `robot_execution_failed_or_not_safe`.
   Motion rejections also did not explicitly pause the in-flight turn. The app
   now retains the causal rejection/failure, side, arm, move and stage. A generic
   blocked-state heartbeat cannot overwrite it. Qt distinguishes pre-motion
   rejection from execution requiring reconciliation; labels wrap visibly.
3. **Unbounded-looking waits.** The backend now reports named service/controller/
   gripper operations with operation IDs, stage context and result deadlines.
   The Qt coordinator watches missing acceptance/progress/results asynchronously.
   An ambiguous timeout queries authoritative state **once**, preserves the
   request/in-flight guard, and stays paused even if a snapshot appears idle.
   It never resends a move automatically. A normal delayed vision confirmation
   produces a visible notice, keeps the confirmed board unchanged and allows
   progression only after the existing Chess Brain commits the known plan from
   fresh vision. Late events for already confirmed plans cannot put the display
   back into an old acknowledgement wait.

The GUI now shows, for example, `White / robot1 g1f3 • pickup preflight •
Rejected before arm movement • No collision-free descent ...`, instead of an
unexplained pending move. While moving it shows both stage and operation, such
as `Black / robot2 ... • transfer • controller execution result`. The original
vision nodes, confirmation logic, ROS message types/command interfaces, geometry,
limits, speeds and collision requirements were preserved. Motion JSON gains
additive diagnostic context/operation statuses; no topic or message definition
was replaced.

## Negative and wait-path tests

- **Delayed vision:** hold existing mock vision after `e2e4`'s successful robot
  acknowledgement for over 30 seconds. Qt shows “Vision confirmation delayed”,
  FEN/history remain initial, move input stays disabled, and no Stockfish reply
  is requested. Releasing the existing parameter commits the human move, then
  exactly one `d7d5` reply commits and human control returns. See
  [delayed result](stall_validation/delayed/qt_e2e4.json) and
  [held screenshot](stall_validation/delayed/qt_e2e4_held.png).
- **Gripper failure:** the existing `fail_next` parameter produces a visible
  gripper failure, zero committed moves and no engine dispatch. See
  [failure result](stall_validation/gripper_failure/qt_e2e4.json).
- **Missing command result:** the test driver deliberately drops one outgoing
  human command to simulate an ambiguous transport outcome. The app times out
  after 10 seconds, requests `/chess/state`, displays the reconciliation message,
  and stays disabled after the idle snapshot. Repeated Submit clicks cause no
  second dispatch. No joint movement or confirmed-board change occurs. This is
  explicit fault injection, not a claim of observing a natural dropped DDS
  message. See [timeout result](stall_validation/timeout/qt_e2e4.json).
- [Final knight rejection](stall_validation/final/qt_g1f3.json) and
  [screenshot](stall_validation/final/qt_g1f3.png) verify the final visible message
  and zero joint travel. All these app-owned launches/workers closed cleanly.

## Checks actually run

- Full workspace build: **9 packages**.
- Direct repository pytest: **187 passed**, including new branch-preflight,
  no-movement-on-rejection, named timeout, causal-error retention, missing-result,
  late-vision reconciliation, duplicate-dispatch and Black-reply context tests.
- Colcon with explicit pytest: **161 tests, zero failures/errors/skips**. The
  additional description-package tests are covered by direct pytest.
- Static package/launch/Xacro audit: **OK**.
- All four openings through Qt and direct ROS, before and after; live contact
  query; delayed vision, gripper failure and dropped-command timeout probes.
- No observed Qt event-loop gap exceeded 0.1 seconds in these recorded cases.
  No dependencies blocked live reproduction. No hardware, camera, native-window
  interaction or Gazebo physics validation is claimed for this task.

Evidence: [pytest](stall_validation/pytest.txt),
[colcon](stall_validation/colcon_results.txt), [build](stall_validation/build.txt),
[static audit](stall_validation/static_audit.txt).

## Rebuild and repeat

From `Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
./scripts/run_desktop.sh
```

Rebuild before starting: existing copied Python packages under `install` can
otherwise shadow source edits. The installed backend and coordinator were
checked for the new methods during this verification.

For the diagnostic probes, close any backend using these test domains first:

```bash
export PYTHONPATH="$PWD/.qt-deps:$PYTHONPATH"
export QT_QPA_PLATFORM=offscreen
python3 src/dual_scara_desktop/tests/gui_opening_probe.py \
  --workspace "$PWD" --domain 221 --transport qt --tag repeat_qt
python3 src/dual_scara_desktop/tests/gui_opening_probe.py \
  --workspace "$PWD" --domain 225 --transport ros --tag repeat_ros
python3 src/dual_scara_desktop/tests/gui_opening_probe.py \
  --workspace "$PWD" --domain 229 --moves e2e4 --hold-vision --tag repeat_vision
python3 src/dual_scara_desktop/tests/gui_opening_probe.py \
  --workspace "$PWD" --domain 230 --moves e2e4 --fail-gripper --tag repeat_failure
python3 src/dual_scara_desktop/tests/gui_opening_probe.py \
  --workspace "$PWD" --domain 231 --moves e2e4 --drop-command --tag repeat_timeout
python3 -m pytest -q src
```

The opening probe collects each outcome rather than asserting that every legal
chess move is mechanically executable; inspect each report's `complete`, reason,
confirmed plies and joint-travel fields. The default `g1` moves remain expected
collision rejections. Making those physically executable requires a separately
validated grasp/TCP/board/mount/obstacle change, not a planner bypass.

## Files changed for this fix

- `src/chess_motion_interface/chess_motion_interface/backend.py`
- `src/chess_motion_interface/chess_motion_interface/node.py`
- `src/chess_motion_interface/tests/test_geometry_operations.py`
- `src/chess_motion_interface/tests/test_descent_preflight.py` (new)
- `src/dual_scara_desktop/dual_scara_desktop/coordinator.py`
- `src/dual_scara_desktop/dual_scara_desktop/controller.py`
- `src/dual_scara_desktop/dual_scara_desktop/window.py`
- `src/dual_scara_desktop/tests/test_coordinator.py`
- `src/dual_scara_desktop/tests/gui_opening_probe.py` (new)
- This report, its `stall_validation/` evidence, and a link in the desktop guide.
