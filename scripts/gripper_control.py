import sys
import time

USE_SIM = "--sim" in sys.argv

if USE_SIM:
    # Sim gripper via ros2_control
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray

    class SimGripper(Node):
        def __init__(self):
            super().__init__('gripper_controller_node')
            self.publisher = self.create_publisher(
                Float64MultiArray,
                '/gripper_controller/commands',
                10
            )
            time.sleep(1)
            self.get_logger().info('Sim gripper ready!')

        def _send(self, position: float):
            msg = Float64MultiArray()
            msg.data = [position]
            self.publisher.publish(msg)

        def open(self):
            self._send(0.0)
            self.get_logger().info('Gripper open')
            time.sleep(2)

        def close(self):
            self._send(0.8)
            self.get_logger().info('Gripper closed')
            time.sleep(2)

        def partial(self, fraction: float):
            pos = max(0.0, min(0.8, fraction * 0.8))
            self._send(pos)
            self.get_logger().info(f'Gripper at {fraction*100:.0f}%')
            time.sleep(2)

    def main():
        rclpy.init()
        gripper = SimGripper()
        gripper.open()
        time.sleep(1)
        gripper.close()
        time.sleep(1)
        gripper.partial(0.5)
        time.sleep(1)
        gripper.open()
        rclpy.shutdown()

else:
    # Real gripper via Modbus TCP through UR controller
    import asyncio
    from pymodbus.client import AsyncModbusTcpClient

    GRIPPER_HOST = "192.168.56.101"
    GRIPPER_PORT = 63352

    async def send_gripper_command(position: int, speed: int = 255, force: int = 255):
        """position: 0 = fully open, 255 = fully closed"""
        async with AsyncModbusTcpClient(GRIPPER_HOST, port=GRIPPER_PORT) as client:
            await client.write_registers(
                0x03E8,
                [
                    0x0900,
                    0x0000,
                    (speed << 8) | force,
                    position << 8,
                ],
                slave=9
            )

    async def open_gripper():
        print("Opening gripper...")
        await send_gripper_command(position=0)
        print("Gripper open!")

    async def close_gripper():
        print("Closing gripper...")
        await send_gripper_command(position=255)
        print("Gripper closed!")

    async def partial_gripper(fraction: float):
        position = int(fraction * 255)
        print(f"Moving gripper to {fraction*100:.0f}%...")
        await send_gripper_command(position=position)

    def main():
        print("Testing gripper...")
        asyncio.run(open_gripper())
        time.sleep(2)
        asyncio.run(close_gripper())
        time.sleep(2)
        asyncio.run(partial_gripper(0.5))
        time.sleep(2)
        asyncio.run(open_gripper())
        print("Test complete!")

if __name__ == '__main__':
    main()