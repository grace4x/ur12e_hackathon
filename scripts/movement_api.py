import sys
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

USE_SIM = "--sim" in sys.argv

CONTROLLER_TOPIC = (
    "/joint_trajectory_controller/joint_trajectory"
    if USE_SIM
    else "/scaled_joint_trajectory_controller/joint_trajectory"
)

class MovementAPI(Node):
    """
    Unified API for controlling the UR12e arm and Robotiq gripper.
    
    Sim:  python3 movement_api.py --sim
    Real: python3 movement_api.py
    """

    def __init__(self):
        super().__init__('movement_api')

        # Arm publisher
        self.arm_publisher = self.create_publisher(
            JointTrajectory,
            CONTROLLER_TOPIC,
            10
        )

        # Gripper — sim only for now, real hardware uses Modbus TCP
        if USE_SIM:
            self.gripper_publisher = self.create_publisher(
                Float64MultiArray,
                '/gripper_controller/commands',
                10
            )
        else:
            from pymodbus.client import AsyncModbusTcpClient
            import asyncio
            self._modbus_host = "192.168.56.101"
            self._modbus_port = 63352
            self._asyncio = asyncio
            self._AsyncModbusTcpClient = AsyncModbusTcpClient

        time.sleep(1)
        mode = "simulation" if USE_SIM else "real hardware"
        self.get_logger().info(f'MovementAPI ready — {mode}')

    # -------------------------------------------------------------------------
    # Arm
    # -------------------------------------------------------------------------

    def move_to(self, positions: list, time_sec: int = 3):
        """
        Move arm to joint positions.
        positions: list of 6 floats [shoulder_pan, shoulder_lift, elbow,
                                      wrist_1, wrist_2, wrist_3] in radians
        """
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
        self.arm_publisher.publish(msg)
        self.get_logger().info(f'Arm moving to {positions}')
        time.sleep(time_sec + 0.5)  # wait for motion to complete

    def home(self):
        """Move arm to home position."""
        self.move_to([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])

    # -------------------------------------------------------------------------
    # Gripper
    # -------------------------------------------------------------------------

    def gripper_open(self):
        """Fully open the gripper."""
        self._send_gripper(0.0 if USE_SIM else 0)
        self.get_logger().info('Gripper open')
        time.sleep(2)

    def gripper_close(self):
        """Fully close the gripper."""
        self._send_gripper(0.8 if USE_SIM else 255)
        self.get_logger().info('Gripper closed')
        time.sleep(2)

    def gripper_partial(self, fraction: float):
        """
        Move gripper to partial position.
        fraction: 0.0 = open, 1.0 = fully closed
        """
        if USE_SIM:
            self._send_gripper(max(0.0, min(0.8, fraction * 0.8)))
        else:
            self._send_gripper(int(fraction * 255))
        self.get_logger().info(f'Gripper at {fraction*100:.0f}%')
        time.sleep(2)

    def _send_gripper(self, position):
        if USE_SIM:
            msg = Float64MultiArray()
            msg.data = [float(position)]
            self.gripper_publisher.publish(msg)
        else:
            self._asyncio.run(self._send_modbus(int(position)))

    async def _send_modbus(self, position: int, speed: int = 255, force: int = 255):
        async with self._AsyncModbusTcpClient(
            self._modbus_host, port=self._modbus_port
        ) as client:
            await client.write_registers(
                0x03E8,
                [0x0900, 0x0000, (speed << 8) | force, position << 8],
                slave=9
            )

    # -------------------------------------------------------------------------
    # Pick and place primitives
    # -------------------------------------------------------------------------

    def pick(self, pre_pick: list, pick: list):
        """Move to pre_pick, descend to pick, close gripper, retreat."""
        self.gripper_open()
        self.move_to(pre_pick)
        self.move_to(pick)
        self.gripper_close()
        self.move_to(pre_pick)

    def place(self, pre_place: list, place: list):
        """Move to pre_place, descend to place, open gripper, retreat."""
        self.move_to(pre_place)
        self.move_to(place)
        self.gripper_open()
        self.move_to(pre_place)


def main():
    rclpy.init()
    api = MovementAPI()

    # Home position
    api.home()
    time.sleep(1)

    # Move to a different pose
    api.move_to([1.0, -1.0, 1.0, -1.0, -1.0, 0.5])
    time.sleep(1)

    # Another pose
    api.move_to([0.5, -1.2, 0.8, -0.8, -0.5, 0.3])
    time.sleep(1)

    # Test gripper while arm is out
    api.gripper_close()
    api.gripper_open()

    # Back to home
    api.home()

    rclpy.shutdown()


if __name__ == '__main__':
    main()