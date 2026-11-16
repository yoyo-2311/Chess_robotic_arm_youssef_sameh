# Integration contract: chess brain <-> robot executor (<-> vision, safety)

Reconstructed from the code of `chess_brain_handover` v1.0 / `chess_auto_coordinator` (the `INTEGRATION_CONTRACT.md` mentioned by the brain node was not in the archive) and implemented by `chess_robot_executor`. All messages are `std_msgs/String` carrying compact JSON unless noted. Frames: board frame B (origin = board centre, +x files a->h, +y ranks 1->8, +z up).

## Topics
| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `/chess/move_plan` | String (JSON) | brain -> executor | one move as ordered pick/place actions (schema below) |
| `/chess/robot_ack` | String (JSON) | executor -> brain | result of a plan (schema below) |
| `/chess/safety_ready` | Bool | safety source -> brain, coordinator | brain only issues plans while true. **No publisher exists yet**: the executor can be that source with `publish_safety_ready:=true` (software state, NOT a physical interlock) |
| `/chess/vision/board` | String (JSON) | camera/vision (M7) -> brain, coordinator | board observation used to confirm each move. **Not provided yet** (tests use a simulated camera) |
| `/chess/executor/status` | String (JSON) | executor -> anyone | events and state (below) |
| `/chess/executor/estop` | Bool | operator/safety -> executor | `true` = software emergency stop, `false` = release |
| `/chess/executor/operator_confirm` | Bool | operator -> executor | `true` = the manual step was done (e.g. promotion piece placed) |
| `/chess/executor/recovered` | Bool | operator -> executor | `true` = both arms were parked by hand and grippers opened after an abort |
| `/chess/command`, `/chess/state`, `/chess/events` | String | brain / coordinator | game control, not used by the executor |

## Move plan (`/chess/move_plan`)
```json
{"plan_id":"43b3...","uci":"e5f6","san":"exf6","side":"white","piece":"P","from_sq":"e5","to_sq":"f6",
 "before_fen":"...","expected_fen":"...","captured":"p","capture_sq":"f5","is_castling":false,"promotion":null,
 "actions":[{"kind":"remove","source":"f5","target":null,"piece":null},
            {"kind":"move","source":"e5","target":"f6","piece":null}],
 "schema_version":"1.0"}
```
Executor requirements: `plan_id` (non-empty string), `side` (`white`|`black`), `actions` (non-empty list). Optional but used: `piece` (grasp height of the first move), `captured` (grasp height of the removed piece), `before_fen` (a start-position FEN resets the capture bins).

| `kind` | Fields used | Executor behaviour |
|---|---|---|
| `remove` | `source` | pick the captured piece on `source`, put it into the next free slot of the moving arm's capture bin (captures and en passant: the removed square is `source`) |
| `move` | `source`, `target` | pick on `source`, place on `target`. Castling = two `move` actions (king first, then rook; the rook is `R`/`r` by side) |
| `promote` | `target`, `piece` | pawn on `target` goes to the bin, then the executor WAITS for `/chess/executor/operator_confirm` (an operator places the new piece; the robot has no piece reserve yet) |

Arm assignment: `white` -> `robot1`, `black` -> `robot2` (configurable; matches the layout robot1 at -Y / white side). Unknown action kinds, bad squares, missing fields: rejected (failure ack, nothing moves).

## Robot acknowledgement (`/chess/robot_ack`)
```json
{"plan_id":"43b3...","success":true,"arm_parked":true,"gripper_released":true,
 "reason":null,"detail":"","arm":"robot1","source":"chess_robot_executor"}
```
The brain reads `plan_id`, `success`, `arm_parked`, `gripper_released`; the other fields are for humans. A failed ack makes the brain block the game until the physical state is reset. One ack per plan. **The ack is derived from COMMANDED state** (no encoder/limit-switch/gripper feedback exists yet): it means "the commanded sequence finished", not "the piece is verified in place" - that is the vision's job.
Failure reasons: `malformed_plan`, `unknown_action`, `invalid_action`, `unreachable`, `z_out_of_range`, `bin_full`, `estop`, `estop_or_recovery_required`, `operator_timeout`.
Dry-run (default): the node logs commands, moves nothing and publishes NO ack (an `ack_suppressed_dry_run` status event instead) so the brain is never told that a move physically happened.

## Status (`/chess/executor/status`)
`{"event":"plan_started|plan_done|plan_failed|plan_ignored|wait_operator|operator_confirmed|estop|estop_released|recovered|operator_timeout|ack_suppressed_dry_run", ..., "state":"idle|executing|wait_operator|estop","plan_id":..,"arm":..,"item":n,"items":m,"progress":0..1,"needs_recovery":bool,"parked":{"robot1":bool,"robot2":bool},"bin_counts":{..},"time":s}` - events on change, plus the state at 2 Hz.
Duplicate `plan_id`s and plans arriving while busy are ignored (status `plan_ignored`), never acknowledged twice.

## Timing and ordering
One plan in flight at a time (the brain guarantees it; the executor enforces it). The executor ticks at 50 Hz by default; a late tick slows motion, it never skips or accelerates it.
