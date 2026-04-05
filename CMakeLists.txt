cmake_minimum_required(VERSION 3.5)
project(my_robot_t2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


install(
  DIRECTORY config description launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/flip_image_node.py
  scripts/mpu6050_node.py
  scripts/motion_commander_ol.py
  scripts/motion_commander_cl.py
  scripts/motion_test_suite.py
  scripts/assisted_teleop.py
  scripts/nav2_goal_sender.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
