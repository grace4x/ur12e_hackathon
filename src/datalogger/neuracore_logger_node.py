#! /usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import neuracore as nc
from std_srvs.srv import Trigger
import cv2
import numpy as np

# run "ros2 service call /start_recording std_srvs/srv/Trigger" to start recording
# ros2 service call /stop_recording std_srvs/srv/Trigger
# "rm ~/.neuracore/config.json" to reset configs

DATASET_NAME = "UR12e Pick and Place"
DEFAULT_CAMERA_ENABLED = True

class NeuracoreLogger(Node):
    def __init__(self, camera_enabled: bool = DEFAULT_CAMERA_ENABLED):
        super().__init__('neuracore_logger')

        nc.login()

        # Cancel any lingering recording from a previous run
        try:
            nc.stop_recording()
        except Exception:
            pass

        robot = nc.connect_robot(
            robot_name="UR12e",
            urdf_path="/home/rosdev/ros2_ws/ur12e_absolute_paths.urdf",
            overwrite=True,
        )

        nc.create_dataset(
            name=DATASET_NAME,
            description="Policy training on picking up and placing",
        )

        print(f"Connected to robot: {robot.id}")
        print(f"Organisation ID: {nc.get_current_org()}")

        self.create_service(Trigger, 'start_recording', self.handle_start_recording)
        self.create_service(Trigger, 'stop_recording', self.handle_stop_recording)

        # Subscribe to actual joint states (what the robot IS doing)
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        # Subscribe to commands (what you're TELLING it to do — needed for training data)
        self.create_subscription(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            self.joint_command_callback,
            10,
        )

        self.cap = None
        self.camera_enabled = camera_enabled
        self.create_timer(1/30, self.image_callback)

        self.recording = False
        self.get_logger().info('Neuracore UR12e logger ready.')
        self.get_logger().info(f'Camera logging: {"ON" if self.camera_enabled else "OFF"}')

    def start_recording(self):
        if self.camera_enabled:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.get_logger().warn('Camera could not be opened — logging without camera.')
                self.cap = None
            else:
                self.get_logger().info('Camera opened.')
        nc.start_recording()
        self.recording = True
        self.get_logger().info('Recording started.')

    def stop_recording(self):
        nc.stop_recording()
        self.recording = False
        if self.cap:
            self.cap.release()
            self.cap = None
        self.get_logger().info('Recording stopped — uploading to Neuracore.')

    def handle_start_recording(self, request, response):
        self.start_recording()
        response.success = True
        response.message = 'Recording started.'
        return response

    def handle_stop_recording(self, request, response):
        self.stop_recording()
        response.success = True
        response.message = 'Recording stopped.'
        return response

    def joint_state_callback(self, msg: JointState):
        if not self.recording:
            return
        t = time.time()
        positions = {name: float(val) for name, val in zip(msg.name, msg.position)}
        velocities = {name: float(val) for name, val in zip(msg.name, msg.velocity)}
        nc.log_joint_positions(positions, robot_name="UR12e", timestamp=t)
        nc.log_joint_velocities(velocities, robot_name="UR12e", timestamp=t)

    def joint_command_callback(self, msg: JointTrajectory):
        if not self.recording or not msg.points:
            return
        t = time.time()
        target_positions = {name: float(val) for name, val in zip(msg.joint_names, msg.points[0].positions)}
        nc.log_joint_target_positions(target_positions, robot_name="UR12e", timestamp=t)

    def image_callback(self):
        if not self.recording or not self.camera_enabled or self.cap is None:
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        nc.log_rgb(name="laptop_cam", rgb=frame_rgb, timestamp=time.time())


def main():
    rclpy.init()
    node = NeuracoreLogger(camera_enabled=DEFAULT_CAMERA_ENABLED)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()