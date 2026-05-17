import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class GripperController(Node):
    """
    Controls the Robotiq 2F-140 gripper via ROS2 topics.
    The gripper node must be running before using this class.
    Start it with: ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py fake_hardware:=true
    For real hardware, remove fake_hardware:=true and make sure gripper is connected via USB.
    """

    def __init__(self):
        # Initialise the ROS2 node with the name 'gripper_controller'
        super().__init__('gripper_controller')

        # Create a publisher that sends commands to the gripper
        # The gripper listens on this topic for open/close commands
        # Float32MultiArray is just a list of floats — we send [1.0] to open, [-1.0] to close
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/robotiq_2f_gripper/binary_command',
            10  # Queue size — how many messages to buffer
        )

        # Wait for the publisher to fully connect before sending any commands
        # Without this, the first command might be lost
        time.sleep(1)
        self.get_logger().info('Gripper controller ready!')

    def open(self):
        """
        Opens the gripper fully.
        Sends 1.0 to the binary_command topic which triggers a full open.
        """
        msg = Float32MultiArray()
        msg.data = [1.0]  # 1.0 = open command
        self.publisher.publish(msg)
        self.get_logger().info('Gripper opening...')
        time.sleep(2)  # Wait for gripper to physically complete the motion
        self.get_logger().info('Gripper open!')

    def close(self):
        """
        Closes the gripper fully.
        Sends -1.0 to the binary_command topic which triggers a full close.
        """
        msg = Float32MultiArray()
        msg.data = [-1.0]  # -1.0 = close command
        self.publisher.publish(msg)
        self.get_logger().info('Gripper closing...')
        time.sleep(2)  # Wait for gripper to physically complete the motion
        self.get_logger().info('Gripper closed!')

    def partial(self, position):
        """
        Moves the gripper to a partial position using confidence command.
        position: float between -1.0 (fully closed) and 1.0 (fully open)
        Example: partial(0.5) opens gripper halfway
        """
        msg = Float32MultiArray()
        msg.data = [float(position)]
        self.publisher.publish(msg)
        self.get_logger().info(f'Gripper moving to position {position}...')
        time.sleep(2)


# This block only runs if you execute this file directly
# i.e. python3 gripper_control.py
# It won't run if you import this file from another script
if __name__ == "__main__":
    # Initialise ROS2 communication
    rclpy.init()

    # Create our gripper controller node
    gripper = GripperController()

    # Test sequence — open, close, open
    print("Testing gripper...")

    print("Opening gripper...")
    gripper.open()

    print("Closing gripper...")
    gripper.close()

    print("Opening gripper halfway...")
    gripper.partial(0.5)

    print("Opening gripper fully...")
    gripper.open()

    print("Test complete!")

    # Shut down ROS2 cleanly
    rclpy.shutdown()