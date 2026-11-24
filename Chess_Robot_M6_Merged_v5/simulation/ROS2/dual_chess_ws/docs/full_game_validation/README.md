# Full-game and logic checks — 23 November 2026

These checks extend the desktop verification. No production motion, geometry,
camera, rules or confirmation behavior was changed; only test drivers, an audit
utility and evidence were added. No commit or push was made.

| Check | Actual result |
|---|---|
| Complete Stockfish logic game, strong White vs easy Black | **PASS:** White checkmate, `1-0`, 53 plies (27 moves), 15 captures, one castling, 27.13 seconds. No errors or ply cutoff. |
| Complete deterministic game through Qt → ROS → MoveIt → mock controllers/gripper → mock vision | **PASS:** `1. f3 e5 2. g4 Qh4#`, `0-1`, four executed and freshly vision-confirmed plies, arm order robot1/robot2/robot1/robot2. About 105 seconds. |
| Unrestricted Stockfish match through the Qt match mode | **BLOCKED:** `e2e4 d7d5` committed. White's next move `d2d3` failed during transfer: `cartesian_path_incomplete: fraction=0.7895 code=1`. The next engine reply was not requested. Confirmed board/history stayed at two plies, the failed move remained pending, reconciliation was required, and shutdown completed cleanly. **This is not a completed Stockfish robot game.** |
| Logic/turn-coordinator regression tests | **37 passed**, including six new complete-game/terminal-state cases. |
| Entire repository test suite | **177 passed**. |

The short deterministic game demonstrates a real terminal result through the
entire mock motion pipeline; it is not evidence that arbitrary positions or a
long game are collision-free. The Stockfish logic game uses the existing
`selfplay` utility with explicitly synthetic acknowledgements and observations;
it does not execute ROS trajectories. The Qt probes never fabricate robot
acknowledgements or vision: those come from the existing motion and mock vision
nodes. The Stockfish Qt probe uses another engine only to operate the human-side
UCI input; the desktop remains the sole automatic opponent coordinator.

## Evidence

- [Full engine game PGN](stockfish_logic/game.pgn), [move CSV](stockfish_logic/moves.csv), [self-play summary](stockfish_logic/summary.json), [replay audit](stockfish_logic/audit.json).
- [Complete scripted robot-game result](scripted.json), [Qt screenshot](scripted.png), [motion events](scripted_motion_log.txt), [Qt test log](scripted_qt.txt).
- [Stockfish robot-game failure result](stockfish.json), [failure screenshot](stockfish.png), [motion failure trace](stockfish_motion_log.txt), [Qt test log](stockfish_qt.txt).
- [Focused logic tests](logic_tests.txt), [all tests](all_tests.txt).

Replay auditing checks every UCI move against the preceding position, SAN, both
FENs, side alternation, unique plan/capture IDs, actual terminal outcome and the
absence of moves after game over. A ply limit is rejected as incomplete. New
regression cases cover White/Black checkmate, threefold repetition, stalemate,
insufficient material and the fifty-move rule. Complete sequences also assert
that old/pre-ack vision and duplicate acknowledgements cannot commit a move,
duplicate post-commit observations cannot add another ply, and game-over rejects
further plans. Draw behavior follows the existing brain's
`outcome(claim_draw=True)` policy: an available claim is taken automatically.

The complete Qt game checks the displayed board against replayed confirmed
history, successful robot acknowledgement and vision-commit counts, correct
arms, disabled input after checkmate, unchanged automatic-request keys after
game over, responsive Qt timers and clean worker/process shutdown. Maximum
observed Qt timer gaps were approximately 37 ms in these runs.

## Repeat the tests

From `Chess_Robot_M6_Merged_v5/simulation/ROS2/dual_chess_ws`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/.qt-deps:$PYTHONPATH"
export QT_QPA_PLATFORM=offscreen

# Complete deterministic terminal game with real mock-controller execution:
python3 src/dual_scara_desktop/tests/gui_full_game_probe.py \
  --workspace "$PWD" --domain 219 --mode scripted

# Free-playing Stockfish robot test; exits nonzero if motion blocks the game:
python3 src/dual_scara_desktop/tests/gui_full_game_probe.py \
  --workspace "$PWD" --domain 220 --mode stockfish --max-plies 400

# Complete logic-only self-play, followed by mandatory terminal-game audit:
python3 -u -m chess_brain_handover.selfplay \
  --white strong --black easy --max-plies 600 \
  --engine-path /usr/games/stockfish \
  --output docs/full_game_validation/stockfish_logic
python3 tools/audit_selfplay_game.py docs/full_game_validation/stockfish_logic

python3 -m pytest -q src
```

Do not run these domains alongside another backend using the same domain.
Stockfish strength-limited choices are not deterministic, so later games may
have different moves or hit a different collision first. The scripted game is
repeatable. A pipeline failure is retained as a failed test, never skipped or
treated as a successful move. The drivers close only their owned processes.

## Remaining issue

General Stockfish robot play cannot yet be claimed to work for a full game.
The existing low-board CAD, fixed gripper and collision-constrained transfer
paths need further investigation/revalidation. This test did not change physical
measurements or weaken collision checks. The failed transfer may leave partial
simulated motion, so confirmed FEN alone must not be interpreted as the actual
physical piece location; the backend correctly requires reconciliation.

No real motors, camera or physical board were tested. The initial logic-only
engine run stalled during sandboxed engine initialization; rerunning outside
the sandbox completed normally. The successful Qt/ROS probes also ran outside
the socket-restricted sandbox. All test-owned processes were stopped afterward.
