"""Robot executor for the dual-arm chess robot (pure-Python core + thin ROS 2 node).

Frames: B = board frame (origin board centre, +x a->h, +y rank 1->8, +z up). R = robot base frame (origin on the arm's
vertical base axis, +x = arm 'forward' at joint zero). Units: metres, radians, seconds.
Every numeric default is a PLACEHOLDER until the Design team delivers the final arm (see docs/06_control_architecture.md).
"""
