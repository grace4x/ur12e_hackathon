import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    pkg = get_package_share_directory('ur12e_hackathon')
    xacro_file = os.path.join(pkg, 'urdf', 'ur12e_with_gripper.urdf.xacro')
    robot_description = {'robot_description': xacro.process_file(xacro_file).toxml()}
    controllers_yaml = os.path.join(pkg, 'config', 'ros2_controllers.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'sim.rviz')
    display = os.environ.get('DISPLAY', 'host.docker.internal:0')

    print(f'Looking for rviz config at: {rviz_config}')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers_yaml],
    )

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    spawn_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
    )

    spawn_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[robot_description],
        additional_env={'DISPLAY': display},
    )

    return LaunchDescription([
        rsp,
        ros2_control_node,
        TimerAction(period=2.0, actions=[spawn_jsb]),
        TimerAction(period=3.0, actions=[spawn_arm]),
        TimerAction(period=3.0, actions=[spawn_gripper]),
        TimerAction(period=4.0, actions=[rviz]),
    ])