cmake_minimum_required(VERSION 3.5)
project(laser_go)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_executable(laser_go src/laser_go.cpp)
add_executable(laser_go2 src/laser_go2.cpp)

ament_target_dependencies(laser_go rclcpp sensor_msgs geometry_msgs std_msgs)
ament_target_dependencies(laser_go2 rclcpp sensor_msgs geometry_msgs std_msgs)

install(TARGETS
  laser_go
  laser_go2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
