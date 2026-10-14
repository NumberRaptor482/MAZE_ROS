cmake_minimum_required(VERSION 3.8)
project(serializer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(serializer src/serializer.cpp)
ament_target_dependencies(serializer rclcpp sensor_msgs)

install(TARGETS serializer
        DESTINATION lib/${PROJECT_NAME})

ament_package()