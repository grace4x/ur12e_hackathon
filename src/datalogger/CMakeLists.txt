cmake_minimum_required(VERSION 3.8)
project(datalogger)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  neuracore_logger_node.py
  policy_runner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

