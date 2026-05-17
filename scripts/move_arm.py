import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import sys

# Set USE_SIM=False when running on real hardware
USE_SIM = "--sim" in sys.argv

CONTROLLER_TOPIC = (
    "/joint_trajectory_controller/joint_trajectory"
    if USE_SIM
    else "/scaled_joint_trajectory_controller/joint_trajectory"
)

class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')
        self.publisher = self.create_publisher(
            JointTrajectory,
            CONTROLLER_TOPIC,
            10
        )
        self.get_logger().info(f'ArmMover ready on topic: {CONTROLLER_TOPIC}')

    def move_to(self, positions, time_sec=3):
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=time_sec)
        msg.points = [point]
        self.publisher.publish(msg)
        self.get_logger().info(f'Moving to {positions}')

def main():
    rclpy.init()
    node = ArmMover()
    time.sleep(1)

    node.move_to([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
    time.sleep(4)

    node.move_to([1.0, -1.0, 1.0, -1.0, -1.0, 0.5])
    time.sleep(4)

    rclpy.shutdown()

if __name__ == '__main__':
    main()