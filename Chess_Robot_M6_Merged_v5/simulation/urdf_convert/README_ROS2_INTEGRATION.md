# About this folder
Untouched SolidWorks -> URDF export of `Assem4.SLDASM` (ROS 1 / catkin, `package://Assem4.SLDASM/...` mesh URIs, `.launch` XML files). It is **not** built or launched with ROS 2.
The ROS 2 workspace is `../ROS2/dual_chess_ws/` (meshes copied byte-identically to `src/dual_chess_description/meshes/robot_arm/`).
Review of this export: `../ROS2/dual_chess_ws/docs/04_design_urdf_review.md`. Validate a new export with
`python3 ../ROS2/dual_chess_ws/tools/validate_design_urdf.py urdf/<file>.urdf --mesh-root .` before integrating it.
