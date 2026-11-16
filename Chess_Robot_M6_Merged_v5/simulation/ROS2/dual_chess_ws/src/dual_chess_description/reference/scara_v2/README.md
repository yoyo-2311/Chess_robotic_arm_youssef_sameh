# SCARA V2 — ROS 2 Jazzy / Gazebo Harmonic

## Build and launch
```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher
cd ~/dual_chess
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select scara_v2_ros_package
source install/setup.bash
ros2 launch scara_v2_ros_package gazebo.launch.py
```

This spawns the CAD robot in Gazebo Harmonic. It does **not** implement motor actuation or ROS 2 controllers. The ROS joint_state_publisher provides only visualization TF. The original world-to-base rotation, inertias, limits and full mesh collisions were retained. Check mesh orientation, floor clearance, and collision behavior in Gazebo before controlling it. For actuation add ros2_control and gz_ros2_control, with joint controllers.
