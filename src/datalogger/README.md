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
ros2 run datalogger neuracore_logger_node.py
```

## Running a trained policy

With the robot driver up (no MoveIt — the runner talks straight to
`scaled_joint_trajectory_controller`):

```bash
ros2 run datalogger policy_runner_node.py
ros2 service call /start_policy std_srvs/srv/Trigger
ros2 service call /stop_policy  std_srvs/srv/Trigger
```

By default the model is downloaded from Neuracore using `TRAIN_RUN_NAME`
(edit at the top of `policy_runner_node.py`). To use a local bundle, point
`NC_MODEL_FILE` at a `.nc.zip` archive that is reachable from inside the
container:

```bash
# If you have an unpacked model directory (model.pt, algorithm/, *.json, metadata):
( cd ~/Downloads/model_arm && zip -r ~/policy.nc.zip . )

# Mount it into the container at start time, then:
NC_MODEL_FILE=/home/rosdev/policy.nc.zip ros2 run datalogger policy_runner_node.py
```

Edit `TRAIN_RUN_NAME`, `CAMERA_INDEX`, `TRAINING_HZ`, and `DEVICE` at the
top of `policy_runner_node.py` to match your training job.
