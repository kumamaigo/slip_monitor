cmake_minimum_required(VERSION 3.8)
project(slip_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(slip_monitor_node src/slip_monitor.cpp)
ament_target_dependencies(
  slip_monitor_node
  rclcpp
  can_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

add_executable(detection_node src/detection_node.cpp)
ament_target_dependencies(
  detection_node
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS
  slip_monitor_node
  detection_node
  DESTINATION lib/
)

install(DIRECTORY launch
  DESTINATION share/
)

install(PROGRAMS
  scripts/plot_slip.py
  DESTINATION lib/
)

ament_package()
