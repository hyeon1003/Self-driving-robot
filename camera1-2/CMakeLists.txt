cmake_minimum_required(VERSION 3.16)
project(camera1-2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

#build
add_executable(pub1-2 src/pub1-2.cpp)
ament_target_dependencies(pub1-2 rclcpp sensor_msgs cv_bridge OpenCV)
add_executable(sub_jetson1-2 src/sub_jetson1-2.cpp)
ament_target_dependencies(sub_jetson1-2 rclcpp sensor_msgs OpenCV)
add_executable(sub_wsl1-2 src/sub_wsl1-2.cpp)
ament_target_dependencies(sub_wsl1-2 rclcpp sensor_msgs OpenCV)

# Install
install(TARGETS
  pub1-2
  sub_jetson1-2
  sub_wsl1-2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
