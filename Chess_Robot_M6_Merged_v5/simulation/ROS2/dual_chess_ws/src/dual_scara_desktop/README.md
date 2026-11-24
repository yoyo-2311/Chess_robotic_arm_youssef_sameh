# dual_scara_desktop

Local PySide6/rclpy desktop console. Entry point: `ros2 run dual_scara_desktop desktop`.

From the workspace, install local Qt dependencies with `scripts/install_desktop_dependencies.sh`, build with `colcon build --symlink-install`, and start with `scripts/run_desktop.sh`.

Full guide: [installation, first match, mode controls and verification](../../docs/10_desktop_application.md).

The package separates widgets, ROS communication, subprocess supervision and match coordination. It reuses the existing Chess Brain and motion interfaces. There is no speculative board update, competing automatic coordinator, hardware driver, or web service.
