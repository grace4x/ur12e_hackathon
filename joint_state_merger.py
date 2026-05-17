import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')
        
        self.arm_joints = JointState()
        self.gripper_joints = JointState()
        
        # Track current and target finger position for smooth interpolation
        self.current_finger_pos = 0.0
        self.target_finger_pos = 0.0
        self.interpolation_speed = 0.02  # How fast to interpolate (radians per tick)
        
        self.create_subscription(JointState, '/joint_states', self.arm_callback, 10)
        self.create_subscription(JointState, '/robotiq_2f_gripper/joint_states', self.gripper_callback, 10)
        
        self.publisher = self.create_publisher(JointState, '/merged_joint_states', 10)
        self.create_timer(0.02, self.publish_merged)
        self.get_logger().info('Joint state merger ready!')

    def arm_callback(self, msg):
        self.arm_joints = msg

    def gripper_callback(self, msg):
        # Update target position from gripper node
        if 'finger_joint' in msg.name:
            idx = msg.name.index('finger_joint')
            self.target_finger_pos = msg.position[idx]

    def publish_merged(self):
        if not self.arm_joints.name:
            return
        
        # Smoothly interpolate finger position towards target
        diff = self.target_finger_pos - self.current_finger_pos
        if abs(diff) > self.interpolation_speed:
            self.current_finger_pos += self.interpolation_speed * (1 if diff > 0 else -1)
        else:
            self.current_finger_pos = self.target_finger_pos
            
        joint_dict = {}
        
        for name, pos in zip(self.arm_joints.name, self.arm_joints.position):
            joint_dict[name] = pos
        
        # Use interpolated finger position
        finger_pos = self.current_finger_pos
        joint_dict['finger_joint'] = finger_pos
        joint_dict['left_inner_knuckle_joint'] = -finger_pos
        joint_dict['right_inner_knuckle_joint'] = -finger_pos
        joint_dict['right_outer_knuckle_joint'] = -finger_pos
        joint_dict['left_inner_finger_joint'] = finger_pos
        joint_dict['right_inner_finger_joint'] = finger_pos
        
        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.name = list(joint_dict.keys())
        merged.position = list(joint_dict.values())
        merged.velocity = []
        merged.effort = []
        
        self.publisher.publish(merged)

def main():
    rclpy.init()
    node = JointStateMerger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()