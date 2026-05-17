import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

    def move_to(self, positions, time_sec=3):
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=time_sec)
        msg.points = [point]
        self.publisher.publish(msg)
        self.get_logger().info(f'Moving to {positions}')

rclpy.init()
node = ArmMover()

# Wait for publisher to connect
time.sleep(2)

# Position 1
node.move_to([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
time.sleep(4)

# Position 2
node.move_to([1.0, -1.0, 1.0, -1.0, -1.0, 0.5])
time.sleep(4)

rclpy.shutdown()