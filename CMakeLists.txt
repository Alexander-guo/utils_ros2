cmake_minimum_required(VERSION 3.8)
project(utils_ros2)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ==============================================================================
# Dependencies
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# ==============================================================================
# Include directories
# ==============================================================================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ==============================================================================
# Library
# ==============================================================================
add_library(${PROJECT_NAME}
  src/Utils.cpp
  src/Trajectory.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  OpenCV
  tf2_ros
  tf2
  rosbag2_cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE -march=native)

# ==============================================================================
# Executable
# ==============================================================================
add_executable(write_kf_images src/write_keyframe_image.cpp)
target_link_libraries(write_kf_images ${PROJECT_NAME})


ament_target_dependencies(write_kf_images
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  OpenCV
)

# ==============================================================================
# Installation
# ==============================================================================
install(TARGETS
  ${PROJECT_NAME}
  write_kf_images
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ==============================================================================
# ament package
# ==============================================================================
ament_package()