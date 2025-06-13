cmake_minimum_required(VERSION 3.16)
project(line_tracer_jetson)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake      REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(std_msgs          REQUIRED)
find_package(sensor_msgs       REQUIRED)
find_package(cv_bridge         REQUIRED)
find_package(OpenCV            REQUIRED)

include_directories(
  include
  /usr/local/include/dynamixel_sdk
)

# 1) 영상 퍼블리셔 전용 노드
add_executable(pub src/pub.cpp)
ament_target_dependencies(pub
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# 2) 에러 구독 → DXL 제어 전용 노드
add_executable(sub src/sub.cpp src/dxl.cpp)
ament_target_dependencies(sub
  rclcpp
  std_msgs
)
target_link_libraries(sub
  dxl_x64_cpp
)

# Install
install(
  TARGETS
    pub
    sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
