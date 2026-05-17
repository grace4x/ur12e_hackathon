import neuracore as nc

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