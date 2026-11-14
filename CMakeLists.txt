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

# 1. slip_monitor_node のビルド設定
add_executable(slip_monitor_node src/slip_monitor.cpp)
ament_target_dependencies(slip_monitor_node
  rclcpp
  can_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

# 2. detection_node のビルド設定
add_executable(detection_node src/detection_node.cpp)
ament_target_dependencies(detection_node
  rclcpp
  can_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

# 3. moving_nodeのビルド設定
add_executable(moving_node src/moving_node.cpp)
ament_target_dependencies(moving_node rclcpp geometry_msgs nav_msgs)

install(TARGETS
  slip_monitor_node
  moving_node
  DESTINATION lib/${PROJECT_NAME}
)

# 実行ファイルのインストール先（lib/slip_monitor/ に配置）
install(TARGETS
  slip_monitor_node
  detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch ファイルと Python スクリプトのインストール
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/plot_slip.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
