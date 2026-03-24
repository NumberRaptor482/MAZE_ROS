cmake_minimum_required(VERSION 3.8)
project(lidar_udp_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(udp_to_laserscan src/udp_to_laserscan.cpp)
ament_target_dependencies(udp_to_laserscan rclcpp sensor_msgs)

install(TARGETS udp_to_laserscan
  DESTINATION lib/${PROJECT_NAME})

ament_package()
