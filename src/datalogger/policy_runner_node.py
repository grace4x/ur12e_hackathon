#! /usr/bin/env python3
"""
Policy runner for UR12e using a Neuracore DiffusionPolicy.

  - Perception timer logs RGB frames (BGR->RGB), joint positions and joint
    velocities to Neuracore. policy.predict() consumes the latest sync point
    of these logs each call.
  - Inference thread runs policy.predict() in a loop and streams successive
    trajectories to scaled_joint_trajectory_controller via FollowJointTrajectory.
  - Receding horizon: each prediction is HORIZON waypoints @ TRAINING_HZ. The
    number of waypoints we skip from the head of each new prediction is
    *dynamic*, based on how long predict() actually took, so we never tell the
    robot to move backwards in time.

By default the policy connects to a deployed Neuracore endpoint, so no local
torch / GPU is needed. Set NC_ENDPOINT_NAME to the deployed endpoint name (or
edit ENDPOINT_NAME below). For in-process inference instead (requires torch
in the container), set NC_TRAIN_RUN_NAME or NC_MODEL_FILE.

Run alongside the robot driver (no MoveIt):
  ros2 launch ur_robot_driver ur_control.launch.py \
      ur_type:=ur12e use_mock_hardware:=false headless_mode:=true \
      robot_ip:=192.168.56.101 \
      kinematics_params_file:="${PWD}/my_robot_calibration.yaml"
"""

import os
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Workaround: neuracore 11.0.0 expects this symbol at the top level of
# neuracore_types, but neuracore-types 7.4.0 only exports it from a submodule.
import neuracore_types as _nct
from neuracore_types.batched_nc_data import DATA_TYPE_TO_BATCHED_NC_DATA_CLASS as _D
from neuracore_types.batched_nc_data.batched_nc_data import BatchedNCData as _B
_nct.DATA_TYPE_TO_BATCHED_NC_DATA_CLASS = _D
_nct.BatchedNCData = _B

import neuracore as nc
from neuracore_types import DataType

ROBOT_NAME            = "UR12e"

# Default: remote endpoint deployed on Neuracore (no local torch / GPU needed).
# Override with NC_ENDPOINT_NAME, or fall back to in-process inference via
# NC_TRAIN_RUN_NAME / NC_MODEL_FILE (both require torch installed locally).
ENDPOINT_NAME         = "test2"                      # deployed Neuracore endpoint
TRAIN_RUN_NAME        = "test diffusion_1_1_1_1_1_1_1"
DEVICE                = "cuda"                       # used only for in-process modes

CAMERA_INDEX          = 4
CAMERA_NAME           = "laptop_cam"

TRAINING_HZ           = 30.0       # camera-bound logging rate in the original logger
HORIZON               = 100
DT                    = 1.0 / TRAINING_HZ
TRAJECTORY_DURATION   = HORIZON * DT

PERCEPTION_HZ         = 30.0
STITCH_LEAD_TIME      = 0.05       # first kept waypoint lands this far in the future

# Refuse to start if first predicted waypoint is further than this (rad) from
# the current joint state on any joint.
MAX_START_DELTA_RAD   = 0.35

CONTROLLER_JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

CONTROLLER_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"


class PolicyRunner(Node):
    def __init__(self):
        super().__init__("policy_runner")
        cb = ReentrantCallbackGroup()

        nc.login()
        nc.connect_robot(ROBOT_NAME)

        endpoint_name = os.environ.get("NC_ENDPOINT_NAME", ENDPOINT_NAME)
        model_file    = os.environ.get("NC_MODEL_FILE")
        train_run     = os.environ.get("NC_TRAIN_RUN_NAME")

        if model_file:
            self.get_logger().info(f"In-process inference from archive: {model_file}")
            self.policy = nc.policy(
                model_file=model_file, robot_name=ROBOT_NAME, device=DEVICE
            )
        elif train_run:
            self.get_logger().info(f"In-process inference, downloading: {train_run}")
            self.policy = nc.policy(
                train_run_name=train_run, robot_name=ROBOT_NAME, device=DEVICE
            )
        else:
            self.get_logger().info(f"Connecting to remote endpoint: {endpoint_name}")
            self.policy = nc.policy_remote_server(endpoint_name=endpoint_name)
        self.get_logger().info("Policy ready.")

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {CAMERA_INDEX}")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.joint_state: Optional[JointState] = None
        self.joint_state_lock = threading.Lock()
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10, callback_group=cb
        )

        self.traj_client = ActionClient(
            self, FollowJointTrajectory, CONTROLLER_ACTION, callback_group=cb
        )
        self.get_logger().info("Waiting for trajectory action server...")
        if not self.traj_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"Trajectory action server {CONTROLLER_ACTION} not available")
        self.get_logger().info("Trajectory action server ready.")

        self.create_service(Trigger, "/start_policy", self._start_cb, callback_group=cb)
        self.create_service(Trigger, "/stop_policy",  self._stop_cb,  callback_group=cb)

        self.running = False
        self._goal_handle_lock = threading.Lock()
        self._current_goal_handle = None
        self.inference_thread: Optional[threading.Thread] = None

        self.create_timer(1.0 / PERCEPTION_HZ, self._perception_tick, callback_group=cb)

        self.get_logger().info("Ready. Call /start_policy to begin.")

    def _on_joint_state(self, msg: JointState):
        with self.joint_state_lock:
            self.joint_state = msg

    def get_joint_positions_in_controller_order(self) -> Optional[np.ndarray]:
        with self.joint_state_lock:
            js = self.joint_state
        if js is None:
            return None
        name_to_pos = dict(zip(js.name, js.position))
        try:
            return np.array([name_to_pos[j] for j in CONTROLLER_JOINT_ORDER])
        except KeyError as e:
            self.get_logger().warn(f"Joint {e} missing from /joint_states")
            return None

    def _perception_tick(self):
        t = time.time()
        ok, frame = self.cap.read()
        if ok:
            try:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                nc.log_rgb(name=CAMERA_NAME, rgb=rgb, robot_name=ROBOT_NAME, timestamp=t)
            except Exception as e:
                self.get_logger().warn(f"log_rgb failed: {e}")

        with self.joint_state_lock:
            js = self.joint_state
        if js is not None:
            try:
                positions = {n: float(v) for n, v in zip(js.name, js.position)}
                nc.log_joint_positions(positions, robot_name=ROBOT_NAME, timestamp=t)
                if len(js.velocity) == len(js.name):
                    velocities = {n: float(v) for n, v in zip(js.name, js.velocity)}
                    nc.log_joint_velocities(velocities, robot_name=ROBOT_NAME, timestamp=t)
            except Exception as e:
                self.get_logger().warn(f"joint logging failed: {e}")

    def _start_cb(self, req, res):
        if self.running:
            res.success = False
            res.message = "already running"
            return res

        current = self.get_joint_positions_in_controller_order()
        if current is None:
            res.success = False
            res.message = "no /joint_states yet"
            return res

        try:
            preview, _ = self._predict_in_controller_order()
        except Exception as e:
            res.success = False
            res.message = f"initial predict failed: {e}"
            return res

        delta = float(np.max(np.abs(preview[0] - current)))
        if delta > MAX_START_DELTA_RAD:
            res.success = False
            res.message = (
                f"first predicted waypoint too far from current pose "
                f"(max delta {delta:.3f} rad > {MAX_START_DELTA_RAD} rad). "
                f"Move robot closer to start pose manually first."
            )
            return res

        self.running = True
        self.inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self.inference_thread.start()
        res.success = True
        res.message = "policy started"
        return res

    def _stop_cb(self, req, res):
        self.running = False
        self._cancel_current_goal()
        res.success = True
        res.message = "policy stopping"
        return res

    def _predict_in_controller_order(self):
        """Returns (waypoints[H,6] in CONTROLLER_JOINT_ORDER, predict_duration_s)."""
        t0 = time.monotonic()
        pred = self.policy.predict(timeout=5)
        predict_dt = time.monotonic() - t0

        joint_dict = pred.get(DataType.JOINT_POSITIONS)
        if not joint_dict:
            raise RuntimeError(
                f"prediction missing JOINT_POSITIONS; got keys: {list(pred.keys())}"
            )

        columns = []
        for j in CONTROLLER_JOINT_ORDER:
            if j not in joint_dict:
                raise RuntimeError(
                    f"prediction missing joint {j}; got: {list(joint_dict.keys())}"
                )
            # BatchedJointData.value: (B=1, T=HORIZON, 1)
            t = joint_dict[j].value.detach().cpu().numpy()
            t = np.squeeze(t)
            if t.shape != (HORIZON,):
                raise RuntimeError(f"unexpected joint tensor shape for {j}: {t.shape}")
            columns.append(t)
        return np.stack(columns, axis=1), predict_dt    # (H, 6)

    def _build_trajectory(self, waypoints: np.ndarray, start_offset_s: float) -> JointTrajectory:
        traj = JointTrajectory()
        traj.joint_names = CONTROLLER_JOINT_ORDER
        for i, q in enumerate(waypoints):
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            t = start_offset_s + i * DT
            pt.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t - int(t)) * 1e9),
            )
            traj.points.append(pt)
        return traj

    def _send_trajectory(self, traj: JointTrajectory):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.traj_client.send_goal_async(goal)

        def on_accepted(f):
            try:
                gh = f.result()
            except Exception as exc:
                self.get_logger().warn(f"goal future failed: {exc}")
                return
            with self._goal_handle_lock:
                self._current_goal_handle = gh

        fut.add_done_callback(on_accepted)

    def _cancel_current_goal(self):
        with self._goal_handle_lock:
            gh = self._current_goal_handle
            self._current_goal_handle = None
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass

    def _inference_loop(self):
        self.get_logger().info("Inference loop started.")
        try:
            while self.running:
                try:
                    waypoints, predict_dt = self._predict_in_controller_order()
                except Exception as e:
                    self.get_logger().error(f"predict() failed: {e}")
                    self.running = False
                    break

                # Skip waypoints that already elapsed during predict(), plus a
                # small lead so the first kept point lands in the future.
                skip = int(predict_dt / DT) + int(STITCH_LEAD_TIME / DT)
                if skip >= HORIZON - 1:
                    self.get_logger().warn(
                        f"predict() took {predict_dt:.2f}s, exceeds horizon "
                        f"{TRAJECTORY_DURATION:.2f}s -- stopping"
                    )
                    self.running = False
                    break
                stitched = waypoints[skip:]

                traj = self._build_trajectory(stitched, start_offset_s=STITCH_LEAD_TIME)
                self._send_trajectory(traj)

                self.get_logger().debug(
                    f"predict={predict_dt*1000:.0f}ms skip={skip} "
                    f"remaining_traj={len(stitched)*DT:.2f}s"
                )
                # Loop again immediately -- predict() itself is the cadence.
        finally:
            self._cancel_current_goal()
            self.get_logger().info("Inference loop exited.")

    def destroy_node(self):
        self.running = False
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PolicyRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
