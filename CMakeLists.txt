cmake_minimum_required(VERSION 3.8)
project(second_project)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(goal_publisher src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

add_executable(scan_qos_relay src/scan_qos_relay.cpp)
ament_target_dependencies(scan_qos_relay
  rclcpp
  sensor_msgs
)

install(TARGETS
  goal_publisher
  scan_qos_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  csv
  map
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
