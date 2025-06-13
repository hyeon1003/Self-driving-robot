cmake_minimum_required(VERSION 3.8)
project(lidarsim)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lidarsim_node src/lidarsim.cpp)
ament_target_dependencies(lidarsim_node rclcpp std_msgs)

target_link_libraries(lidarsim_node
  ${OpenCV_LIBS}
)

install(TARGETS
  lidarsim_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()