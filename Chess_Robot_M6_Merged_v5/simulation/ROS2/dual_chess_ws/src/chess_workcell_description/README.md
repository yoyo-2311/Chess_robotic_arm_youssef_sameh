# CAD workcell adapter

Imports `AssemBase.SLDASM.zip` as a static environment, not a replacement ROS
workspace or SCARA model. The archive contains an enclosure and fixed board.
Original exports/provenance live in `reference/`; mesh bytes are unchanged.

- `assembly.py`: mesh decoding and world transforms, independent of Qt, chess,
  vision, controllers and execution.
- `urdf/workcell.urdf`: normalized standalone fixed-link view.
- `urdf/workcell.macro.xacro`: configurable visualization/Gazebo enclosure.
- `config/workcell_sim.yaml`: provisional arm mounting and grid profile.
- Existing `chess_motion_interface` owns world collisions in MoveIt; the robot
  URDF excludes the environment in that mode. Never add both collision owners.

CAD measurements: board 280 × 280 × 10 mm; enclosure 1250 × 450 × 241.585 mm
in normalized axes. Native -Y is up; Rx(-pi/2) normalizes to +Z. Translation
places board bottom at Z=0, top Z=0.01 m; enclosure extends below it. Original
inertias remain in reference, but are omitted from the static adapter because
the exported enclosure COM is outside its mesh. No dynamics were invented.

The mounting circles and robot foot are now measured from the mesh. The profile
seats each physical mounting face at X=±0.400 m, Y=0, Z≈-0.033 m. This requires
base-link origins near Z=0.404521 m because the native CAD origin is not on the
mounting face. Physical yaw and arm identity are unconfirmed: robot1/White=A
and robot2/Black=B are labelled previews, retaining the existing pi/zero yaw.
The board grid borders, TCP calibration and real grasp depths still require
physical measurements. See `docs/14_mount_alignment.md` before motion: all
64 board squares are unreachable with the correctly seated arm's supplied
vertical travel. Earlier off-platform motion successes do not validate it.

`controle_box_refined.STL` retessellates only the original 140mm circular
pockets, without changing their radius or mounting plane. Coarse original
chords caused false interference with the 139.7mm robot foot. Original meshes
remain preserved; canonical visual and collision geometry use the same derived
asset. Board pose updates move board/squares, not the enclosure.

After building and sourcing, from the workspace:

```bash
ros2 launch chess_workcell_description display.launch.py
ros2 run dual_scara_desktop desktop --workspace "$PWD" --geometry-file "$PWD/src/chess_workcell_description/config/workcell_sim.yaml"
```

Select Human vs Stockfish for MoveIt mock-controller execution with existing
mock gripper/vision. Board moves currently reject due to mechanical reach.
See `docs/14_mount_alignment.md` for transforms, checks and limits.

