cmake_minimum_required(VERSION 3.16)
project(line_tracer_wsl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
)

# 빌드: wsl 노드는 두 소스 파일만 사용
add_executable(wsl
  src/wsl.cpp
  src/main.cpp
)
ament_target_dependencies(wsl
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS wsl
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
