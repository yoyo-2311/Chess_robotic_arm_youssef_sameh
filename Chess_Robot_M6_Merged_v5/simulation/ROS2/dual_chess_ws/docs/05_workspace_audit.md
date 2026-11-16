# Audit of the v1.3.3.4 workspace (chess-logic teammate) and merge decision

Reviewed: `dual_chess_ws_v1_3_3_4.zip` (5 packages) against the M6 audited workspace v4 and the Design-team export. Sandbox only (no ROS, Gazebo, RViz, Stockfish).

## 1. Decision
Continue with **v1.3.3.4 as the code line** (superset of v4's arm: base yaw, wrist/gripper, board CAD, Home pose, primitive collision boxes, chess packages) and keep the M6 audit material alongside it (this merge). v4 remains the fallback only if v1.3.3.4 fails on your machine.

## 2. Verified (sandbox)
| Check | Result |
|---|---|
| Unit tests | coordinator 12 passed; brain 25 passed (needs python-chess: vendored); description 26 passed (static only) |
| `static_audit.py` | OK: 5 packages, 22 Python files, 4 xacro variants (85 links / 84 joints / 10 movable) |
| Design chain preserved | `base_link → link0/1/2` identical to the Design URDF (max difference 0.0, 200 random configs, both robots) |
| Moved base inertial | `R·I·Rᵀ` of link0's inertia equals the stored tensor (6e-21) — the duplicated-base defect of the export is handled correctly (rendered/massed once) |
| HOME pose | robot1 vs robot2 clear (0.145 m, collision geometry); arms vs board clear; tool0 at (∓0.003, ∓0.126, 0.202) |
| Reach (position only) | 64/64 square centres inside robot1's tool-tip reach band (0–0.57 m at z 3–6 cm); orientation, limits and collisions NOT considered |

## 3. Not verified
`colcon build`; Gazebo/RViz/bridge behaviour; Stockfish; the claims of earlier "tested" versions in the teammate's notes (no logs in the archive); v1.3.3.4 gripper tuning (declared candidate).

## 4. Findings
| # | Finding | Handling |
|---|---|---|
| 1 | Board CAD: 40 cm frame, 35 cm playing area, 4.375 cm squares vs requirement 40 cm / 5 cm squares | **Open decision** (README §6.1); not changed |
| 2 | Base yaw, wrist, gripper are not in the Design URDF (code comments call base yaw "intended per project owner") | **Needs confirmation**; documented as temporary/unconfirmed |
| 3 | No consumer of `/chess/move_plan`, no source of `/chess/robot_ack`, `/chess/safety_ready`, `/chess/vision/board` → no move can complete | Future work: simulation robot executor (square → IK → joints → gripper → ack) |
| 4 | Arms share the vertical plane x ≈ 0 at HOME; no collision-aware planning; continuous joints without limits | Future work (Design/M4 limits; planner) |
| 5 | Stale package descriptions | **Fixed** |
| 6 | Brain test imported `chess` before the package could add its vendored copy | **Fixed** |
| 7 | python-chess (GPL-3.0-or-later) vendored in a package labelled MIT; `.gitignore` excludes the vendor dir while the build needs internet once | **Documented** (`THIRD_PARTY_NOTICES.md`); decision for the team |
| 8 | Static tests never expand xacro; no docs/changelog evidence; README a stack of version notes | **Fixed** by tools, docs, README rewrite |
| 9 | `use_meshes` arguments are accepted but unused (CAD STL always used) | Left (harmless) |
| 10 | `reference/` files of the brain are not installed (nothing reads them at run time) | Left |

## 5. Recommended next steps
1. Build and run locally; send logs/screenshots; mark verified items in `CHANGELOG.md`.
2. Decide the board (finding 1) and confirm/replace temporary joints (finding 2) with Design.
3. Agree ownership: description/gazebo (M6), chess packages (chess-logic teammate); one Git repository, one version line.
4. Next milestone: simulation robot executor consuming `/chess/move_plan`.
