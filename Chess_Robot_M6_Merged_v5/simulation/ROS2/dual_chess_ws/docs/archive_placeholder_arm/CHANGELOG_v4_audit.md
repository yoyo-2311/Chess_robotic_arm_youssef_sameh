# Changelog

This file records the notable changes to the Dual Chess ROS 2 workspace.

## Board dimensions specified (16 Oct 2026)
- Chessboard fixed at 40 × 40 cm, 8 × 8 squares of 5 cm (`board_square = 0.05`, `board_size = 0.40`); the model already used these numbers, so geometry is unchanged: board box 0.40 × 0.40 × 0.02 m, a1 centre (-0.175, -0.175), h8 centre (0.175, 0.175). Comments/docs no longer call these two values placeholders. Board thickness (0.02 m) and `base_gap` (0.10 m) remain placeholders. M2 to confirm on the physical board.

## Audit and Design-URDF integration
### Changed
- CAD arm macro now reproduces the Design-team URDF (`Assem4.SLDASM`) faithfully: CAD inertials restored (the earlier rounded values were invented), joint origins/axes/meshes verbatim, kinematics identical to the Design file (max transform difference 0.0 over 200 random configurations).
- The extra `base_yaw_joint` (not part of the Design model) is now optional (`with_base_yaw:=true`, default off).
- Duplicated base part of the export (`base_link` == `link0`) removed by default (`dedupe_base_link:=true`); export defects documented in `docs/04_design_urdf_review.md`.
- `gazebo_cad.launch.py` rewritten: runs the simulation (`-r`), bridges `/clock`, `/joint_states` and joint commands (`config/bridge_cad.yaml`), enables the temporary hold controllers (`sim_gazebo:=true`).
- `display_cad.launch.py` got launch arguments; new `dual_chess_bringup/simulation_cad.launch.py` (Gazebo + RViz).
- Documentation updated (README, `meshes/README.md`, review, report addendum, troubleshooting).
### Added
- `tools/validate_design_urdf.py` (checks a Design-team export), `tools/static_audit.py` (workspace consistency), `docs/04_design_urdf_review.md`, `docs/design_urdf_validation_output.txt`.
### Fixed after the first local run of the audited ZIP
- Both CAD arms occupied the same vertical plane and interpenetrated at the zero pose; they are now shifted -/+`arm_lateral_offset` (0.06 m, placeholder) along board X (clearance 51 mm; 0/300 random-configuration collisions).
### Verified locally by the author (2026-10-15/16, audited ZIP v1 and v2)
- `colcon build` (3 packages), `tools/static_audit.py`, `check_urdf`, RViz sliders, Gazebo spawn without `[Err]`, arms hold pose, `/joint_states`.
- v2 (lateral offset): arms side by side in RViz and Gazebo; commands `robot1_joint1` 0.5 -> 0.4999999999999 and `robot2_joint2` -0.5 -> -0.4999999999998 (others ≈ 1e-13); TF tree of 15 frames rooted at `world`. Gazebo real-time factor ≈ 73–77 %.
### Verified (sandbox, static) / Not yet run
- Static and sandbox checks listed in `docs/04_design_urdf_review.md` §5 passed. Only the optional variants (`with_base_yaw`, `dedupe_base_link:=false`, single robot) still need a local run.

## Current Repository Preparation

### Added
- Added the `dual_chess_description` ROS 2 package.
- Added the `dual_chess_gazebo` ROS 2 package.
- Added the `dual_chess_bringup` ROS 2 package.
- Added CAD mesh files for the robot arm.
- Added CAD-specific URDF/Xacro descriptions.
- Added RViz and Gazebo launch configurations for the CAD model.
- Added the Gazebo chess world and bridge configuration.
- Added workspace documentation, verification notes, and screenshots.
- Added build and launch instructions to the workspace README.

### Reported by the author (no logs/screenshots were available to the audit)
- Built the workspace from its source packages.
- Ran the CAD visualization launch in RViz.
- Ran the CAD-specific Gazebo launch.

### Repository Cleanup
- Keep source files and documentation in Git.
- Exclude generated `build/`, `install/`, and `log/` directories.
- Replace the old workspace ZIP with the directly accessible workspace files.

### Notes
This entry documents the current repository preparation. It is not a list of historical Git commits.
