Put datalogger/ in a ros2 workspace.

```bash
cd ~/ros2_ws
mkdir src/
mv datalogger ~/ros2_ws/src
cd ../
colcon build --symlink-install --packages-select datalogger
```
