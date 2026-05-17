cmake_minimum_required(VERSION 3.8)
project(ur12e_hackathon)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch urdf config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(scripts)

install(PROGRAMS
  scripts/move_arm.py
  scripts/gripper_control.py
  scripts/movement_api.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()