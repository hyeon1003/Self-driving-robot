cmake_minimum_required(VERSION 3.16)
project(psub2-1)
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
find_package(std_msgs REQUIRED)
include_directories(include)
#build
add_executable(pub2-1 src/pub2-1.cpp src/pmain2-1.cpp)
ament_target_dependencies(pub2-1 rclcpp std_msgs)
add_executable(sub2-1 src/sub2-1.cpp src/smain2-1.cpp)
ament_target_dependencies(sub2-1 rclcpp std_msgs)
# Install
install(TARGETS
pub2-1
sub2-1
DESTINATION lib/${PROJECT_NAME})
ament_package()