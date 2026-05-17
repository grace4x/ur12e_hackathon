Put datalogger/ in a ros2 workspace.

```bash
cd ~/ros2_ws
mkdir src/
mv datalogger ~/ros2_ws/src
cd ../
colcon build --symlink-install --packages-select datalogger
source install/setup.bash
```

Move ur12e_absolute_paths.urdf to the necessary path.

```bash
ros2 run datalogger neuracore_logger_node
```
