import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import neuracore as nc

# might delete and re generate, but he also said one time setup code should go in __init__

class NeuracoreLogger(Node):
    def __init__(self):
        super().__init__('neuracore_logger')

        nc.login()
        nc.connect_robot(robot_name="UR10e")  # no urdf_path needed after first setup

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
        self.get_logger().info('Neuracore logger ready.')

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
        nc.log_joint_positions("ur10e", positions, timestamp=t)
        nc.log_joint_velocities("ur10e", velocities, timestamp=t)

    def joint_command_callback(self, msg: JointTrajectory):
        if not self.recording or not msg.points:
            return
        t = time.time()
        # Log the target positions being commanded — this is your "action" for training
        target_positions = dict(zip(msg.joint_names, msg.points[0].positions))
        nc.log_joint_target_positions("ur10e", target_positions, timestamp=t)


def main():
    rclpy.init()
    node = NeuracoreLogger()

    # Start a recording session
    node.start_recording()

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
