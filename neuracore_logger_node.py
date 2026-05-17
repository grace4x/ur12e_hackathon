import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import neuracore as nc


class NeuracoreLogger(Node):
    def __init__(self):
        super().__init__('neuracore_logger')

        nc.login()

        robot = nc.connect_robot(
            robot_name="UR12e",
            urdf_path="/home/rosdev/ur12e_abs.urdf",  # from UR's official ROS2 description package
            overwrite=True,
        )

        nc.create_dataset(
            name="UR12e Pick and Place",
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
        
        self.recording = False
        self.get_logger().info('Neuracore UR12e logger ready.')

    def start_recording(self):
        nc.start_recording()
        self.recording = True
        self.get_logger().info('Recording started.')

    def stop_recording(self):
        nc.stop_recording()
        self.recording = False
        self.get_logger().info('Recording stopped — uploading to Neuracore.')

    def joint_state_callback(self, msg: JointState):
        if not self.recording:
            return
        t = time.time()
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        nc.log_joint_positions("ur12e", positions, timestamp=t)
        nc.log_joint_velocities("ur12e", velocities, timestamp=t)

    def joint_command_callback(self, msg: JointTrajectory):
        if not self.recording or not msg.points:
            return
        t = time.time()
        target_positions = dict(zip(msg.joint_names, msg.points[0].positions))
        nc.log_joint_target_positions("ur12e", target_positions, timestamp=t)


def main():
    rclpy.init()
    node = NeuracoreLogger()


    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
