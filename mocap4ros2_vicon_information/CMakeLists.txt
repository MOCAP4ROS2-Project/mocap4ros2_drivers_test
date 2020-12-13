cmake_minimum_required(VERSION 3.5)

project(mocap4ros2_vicon_information)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

set(dependencies
    rclcpp
    tf2
    tf2_ros
    mocap_msgs
    geometry_msgs
    visualization_msgs
)

add_executable(mocap4ros2_vicon_information src/mocap4ros2_vicon_information.cpp)
ament_target_dependencies(mocap4ros2_vicon_information ${dependencies})

install(TARGETS
  mocap4ros2_vicon_information
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()