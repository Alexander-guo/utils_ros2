cmake_minimum_required(VERSION 3.0.2)
project(utils_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS 
            sensor_msgs cv_bridge roscpp rosbag message_filters
            image_transport camera_info_manager)

find_package(Eigen3 REQUIRED)
find_package(OpenCV  REQUIRED)


catkin_package(
		INCLUDE_DIRS include
		CATKIN_DEPENDS sensor_msgs cv_bridge roscpp rosbag message_filters image_transport camera_info_manager
		LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(ros_utils 
            src/Utils.cpp
            src/Trajectory.cpp
            )

target_link_libraries(ros_utils  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_compile_options(ros_utils  PRIVATE -march=native)

add_executable(write_kf_images src/write_keyframe_image.cpp)
target_link_libraries(write_kf_images ros_utils -lstdc++fs)