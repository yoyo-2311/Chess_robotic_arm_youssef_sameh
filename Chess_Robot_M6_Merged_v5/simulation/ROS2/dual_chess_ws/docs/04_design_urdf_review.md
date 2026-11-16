> **Status note (merge with v1.3.3.4):** this review was written for the M6 audited workspace v4. Its findings about the *Design export* (duplicated base part,
> wrong base inertial frame, ROS 1 mesh URIs, 2 DOF, no limits, assembly zero pose) still apply. In v1.3.3.4 the duplicate base is handled differently
> (link0 geometry/mass moved once onto `robotN_base_yaw_link`, inertia correctly rotated), a base-yaw joint, a temporary wrist and a gripper were added,
> and the arms use the supplied board CAD. Sections that mention `cad_chess_arm`, `with_base_yaw`, `dedupe_base_link` or `arm_lateral_offset` describe v4 only. See `05_workspace_audit.md`.

# Review of the Design-team URDF (`Assem4.SLDASM`) and its integration

Source reviewed: `simulation/urdf_convert/` (SolidWorks → URDF export, `sw_urdf_exporter 1.6.0`, ROS 1 catkin package; kept untouched).
Integrated copy: `src/dual_chess_description/urdf/arm/arm_cad.macro.xacro` + `meshes/robot_arm/*.STL` (byte-identical copies of the export's meshes).
Machine-readable evidence: `docs/design_urdf_validation_output.txt` (`tools/validate_design_urdf.py`).

## 1. What the Design URDF contains
| | |
|---|---|
| Links | `base_link`, `link0`, `link1`, `link2` (STL meshes, metres; visual = collision mesh) |
| Joints | `joint0` fixed (base_link→link0) · `joint1` **continuous**, axis Z (link0→link1) · `joint2` **continuous**, axis Z (link1→link2) |
| Actuated DOF | **2** (independent, both revolute; confirmed by two independent parsers) |
| Limits / dynamics | none (continuous joints without `<limit>`, no `<dynamics>`) |
| End effector | none (no gripper, no tool frame) |
| Geometry | link1→link2 axis distance 0.200 m; link1/link2 meshes 260 × 60 × 40 mm; base/link0 block 60 × 60 × 90 mm |
| Mass | 0.137 kg (base block), 0.367 kg per moving link; mass ÷ mesh volume ≈ 1004–1008 kg/m³ for every link (consistent with one printed-plastic-like density) |

At the zero pose both joint axes are **horizontal and antiparallel** in the root frame: the arm is a 2-link arm swinging in one vertical plane
(link1 and link2 point along −Y of the root frame, ≈ 9.9° and ≈ 5.1° above horizontal at zero).

## 2. Structural validity
- XML valid; unique link/joint names; one root (`base_link`); tree, no cycles; all parents/children exist; joint types valid; axes valid; meshes present, binary STL, metric units; inertias positive definite and satisfy the triangle inequality for all four links.  *(Checked with my own parser, `tools/validate_design_urdf.py`, and independently with `yourdfpy`/`trimesh`.)*

## 3. Findings and how each was handled
Classification: **Confirmed bug** (fixed) · **Design decision requiring confirmation** (not changed) · **Future dependency** (placeholder/interface left).

| # | Finding | Class | Handling |
|---|---|---|---|
| D1 | `base_link` and `link0` are the **same SolidWorks component** (`link0-1` in the CSV) exported twice: identical mesh volume at the identical place in the root frame (bounding boxes and centroids coincide; fcl penetration depth 27 mm). Mass (0.137 kg) and collision geometry are counted twice. | Confirmed bug (export artefact) | `dedupe_base_link:=true` (default): `base_link` keeps its frame but drops the duplicate visual/collision/mass; `link0` carries the real part. `false` reproduces the export. |
| D2 | `base_link`'s `<inertial>` is not expressed in the `base_link` frame: COM is **0.369 m** from its own mesh centroid (y/z swapped relative to link0's transformed COM); tensor has link0's orientation. | Confirmed bug (export artefact) | Neutralised by D1 handling; reproduced unchanged (and flagged) only with `dedupe_base_link:=false`. Please have Design re-export. |
| D3 | Mesh URIs `package://Assem4.SLDASM/...` (ROS 1 package that does not exist in this workspace). | Confirmed bug (for ROS 2) | Rewritten to `package://dual_chess_description/meshes/robot_arm/...`. |
| D4 | `joint0` (fixed) carries `<axis xyz="0 0 0">`; links use `<material name="">`. | Harmless / non-conformant | Axis dropped; one colour per robot instead. |
| D5 | The root frame `base_link` is the SolidWorks **assembly origin**, ≈ 0.31 m away from the physical arm (link0 origin at (-0.186, -0.244, -0.127) m). | Property of the export | `<prefix>mount_link` placed exactly at link0's origin (arm base point); the caller's `<origin>` positions that point. Net transform mount→link0 is joint0's rotation only. |
| Q1 | **Only 2 DOF**, in one vertical plane; no base rotation, wrist or gripper; no TCP. With no lateral DOF the tip cannot leave one plane, so it cannot address the chessboard files. | Design decision / future dependency | Not invented. The earlier integration's extra `base_yaw_joint` was **not in the Design model**; now optional (`with_base_yaw:=true`), **off by default**. |
| Q2 | Continuous joints, **no limits** (effort, velocity, position). A 10° sweep of both joints finds `link2` colliding with the base block in 208 of 1369 configurations (15 %). Joint limits are therefore physically necessary. | Future dependency (Design/M4/M9) | Left unlimited; Gazebo stand-in uses a clearly labelled TEMPORARY 1 rad/s cap. |
| Q3 | The zero pose is the **SolidWorks assembly pose**, not a defined home: `joint1` origin pitch 80.1°, `joint2` origin yaw −4.7°, `joint2` roll π (axis flips). | Design decision | Kept verbatim. Need intended zero/home and sign conventions (M3). |
| Q4 | Which axis is "up" and which direction is the arm's "forward" is not stated. Assumed: root +Z up, arm extends along local −Y at zero (it is where the geometry lies). | Design decision | Mount yaw: robot1 π, robot2 0 so the arm's plane points at the board; confirm with M1/M3. |
| Q5 | Adjacent links interpenetrate by ≈ 1.1 mm (link0/link1) and ≈ 1.5 mm (link1/link2) at zero. | Possible design issue | No effect in simulation (self-collision off); check clearances in CAD. |
| Q6 | link1/link2 collision uses the full 10 630-triangle visual STL. | Future dependency | Request low-poly/convex collision meshes (Week 2 plan). |
| Q7 | No actuator, gearbox, cabling or gripper masses are evident (densities of the printed-plastic kind). Total arm ≈ 0.87 kg. | Needs confirmation | Kept; inertials are CAD values, not invented. |

Earlier ROS 2-side integration (`arm_cad.macro.xacro` as uploaded) — corrected:
| # | Issue | Class | Fix |
|---|---|---|---|
| W1 | Added a continuous `base_yaw_joint` (an extra DOF) as if part of the arm. | Confirmed deviation from "do not invent" | Optional, off by default, documented as not-Design. |
| W2 | Replaced the CAD inertials by rounded invented values (0.3 / 0.2 / 0.3 / 0.3 kg, round tensors). | Confirmed bug | CAD inertials restored for link0/1/2. |
| W3 | `gazebo_cad.launch.py`: no `-r` (sim starts paused), no bridge (`/clock`, `/joint_states` missing although `use_sim_time:=true`), no joint hold; horizontal-axis arm would fall under gravity. | Confirmed bug | Rewritten: `-r`, bridge (`config/bridge_cad.yaml`), `sim_gazebo:=true` plugins (JointStatePublisher + temporary hold controllers). |
| W4 | `dual_chess_cad.urdf.xacro` had no Gazebo section / options. | Confirmed gap | Added `sim_gazebo`, `with_base_yaw`, `dedupe_base_link`. |
| W6 | **Both arms overlapped at the zero pose.** Mounted on the same line (x = 0) with the mount yaws π / 0, the two arms lie in the *same vertical plane* and interpenetrate over the board (found in the author's first local RViz/Gazebo run; confirmed with mesh collision: robot1 link1/link2 vs robot2 link1/link2). | Confirmed bug (layout I chose, not the Design model) | Arms shifted −/+`arm_lateral_offset` (0.06 m, **placeholder**, `arm_cad_params.xacro`) along board X. Zero-pose clearance 51 mm; 0 cross-arm collisions in 300 random joint configurations (3 before). Joint axes are parallel to X, so the arms never leave their x-ranges. |
| W5 | README/CHANGELOG/`meshes/README.md` described the old placeholder layout, `Verified` claims without evidence. | Documentation | Updated; claims reworded (see CHANGELOG). |

## 4. Integration decision
The Design URDF is **used as the primary arm**, wrapped in one reusable, prefixed Xacro macro (`cad_chess_arm`), instantiated twice
(`robot1_`, `robot2_`) on the existing board/placement. Link names, joint names, joint types/axes/origins, meshes and CAD inertials are
unchanged; names get the prefix (`robot1_link1`, `robot2_joint2`, …) so TF and joint names never collide. The Design file itself is **not edited**.
The 5-DOF placeholder arm is kept (it still builds and is what the Week 1 evidence refers to) but is no longer the primary path.

## 5. Verification actually performed (non-ROS sandbox; Python, `xacro 2.1.1`, `yourdfpy`, `trimesh`, `python-fcl`)
| Check | Result |
|---|---|
| XML well-formed (URDF, package.xml, SDF, YAML) | pass |
| Xacro expansion, 7 variants (default, `sim_gazebo`, `with_base_yaw`, both, `dedupe_base_link:=false`, one robot, no robots) + 2 placeholder variants | pass |
| URDF structure (`check_urdf_structure.py`): unique names, 1 root, connected, tree | pass; default CAD model 15 links / 14 joints / 4 movable |
| Independent URDF load (`yourdfpy`) of Design URDF and of the integrated model | pass; actuated joints as expected |
| **Kinematic equivalence**: relative transforms `base_link→link0/1/2`, 200 random joint configs × 2 robots × 3 model variants | max difference **0.0** vs the Design URDF |
| Arm base placement | `link0` origin at (0, −0.30, 0) and (0, +0.30, 0); zero-pose far end of link2 at y = +0.126 / −0.126 (over the board), 0.115 m above ground |
| Mesh URIs of the expansion resolve to files; mesh directory installed by CMake | pass |
| Install rules, launch-file references, exec_depends (`tools/static_audit.py`) | pass |
| Python launch files compile; bridge YAML parses | pass |

**Local verification by the author (Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Sim 8.15.0, 15–16 Oct 2026):**
| Check | Result | Evidence |
|---|---|---|
| `colcon build` | pass | 3 packages finished |
| `tools/static_audit.py` | pass | "OK - no problems found" |
| `check_urdf` | pass | root `world`, children `board_link`, `robot1_mount_link`, `robot2_mount_link`; chains to `link2` |
| RViz (`display_cad.launch.py`) | pass | screenshots: CAD arms, 4 sliders move the right arms, Global Status OK; arm overlap at zero pose seen in this first run -> W6 |
| Gazebo (`simulation_cad.launch.py`) | pass | launch log without `[Err]`; `Entity creation successful`; `gz model --list` shows `ground_plane`, `dual_chess_cad`; Gazebo screenshot |
| Joint states / hold | pass | `/joint_states`: `robot{1,2}_joint{1,2}` plus fixed joints, all ≈ 1e-13 after 53 s (arms hold against gravity) |
| Re-run after the W6 offset fix (v2 ZIP) | pass | `docs/figures/cad_rviz_zero_pose_no_overlap.png`, `cad_gazebo_zero_pose.png`: arms side by side, no overlap; launch log without `[Err]` |
| Joint commands | pass | `robot1_joint1` = 0.4999999999999 (cmd 0.5), `robot2_joint2` = -0.4999999999998 (cmd -0.5); all other joints ≈ 1e-13; `docs/figures/cad_gazebo_after_commands.png` |
| TF (`view_frames`, `frames_2026-10-16_12.28.00.pdf`) | pass | 15 frames, one tree rooted at `world`: `board_*`, and `mount_link → base_link → link0 → link1 → link2` for both robots |
| Not run locally | — | `with_base_yaw:=true`, `dedupe_base_link:=false`, single-robot options (sandbox Xacro expansion only). Gazebo real-time factor ≈ 73–77 % |

**Not run in the sandbox (kept for the record; the author ran the items above locally):** `colcon build`, `check_urdf` (urdfdom), `ros2 launch …`, Gazebo world load, model spawn
(`ros_gz_sim create`), `ros_gz_bridge`, `/joint_states`, `/tf`, RViz. These **must be run locally**; commands in `README.md` §5–§8.
Expected if everything works: `check_urdf` prints root `world` with children `board_link`, `robot1_mount_link`, `robot2_mount_link`; Gazebo lists `ground_plane` and `dual_chess_cad`;
`/joint_states` has `robot1_joint1/2`, `robot2_joint1/2`; commanding `/robot1/joint1/cmd_pos` moves only robot 1's first joint.

## 6. Questions for the team (cannot be answered from the files)
1. **M1/Design:** Is the arm really 2-DOF, or is `Assem4` a partial assembly? What provides lateral motion across the board (base yaw? other joint)? Gripper and TCP?
2. **M1/Design:** Intended zero/home pose and joint sign conventions (the export's zero is the SolidWorks assembly pose).
3. **M1/Design:** Which root axis is up; which direction is the arm's forward at zero; is robot 2 a rotated copy or a mirrored arm?
4. **M1/M4/M9:** Joint position, velocity and effort limits (the arm self-collides in ≈ 15 % of the unrestricted joint space).
5. **M1/Design:** Please re-export without the duplicated `link0-1` as `base_link`; add low-poly collision meshes; are actuator/gearbox/gripper masses included?
6. **M3:** Do you want the TF root at SolidWorks `base_link` or at the arm base point (`<prefix>mount_link`)? Kinematic frame/DH conventions to match.
