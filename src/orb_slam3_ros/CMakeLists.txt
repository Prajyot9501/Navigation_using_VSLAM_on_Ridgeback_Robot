cmake_minimum_required(VERSION 3.0.2)
project(orb_slam3_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# Using relative path to ORB_SLAM3
set(ORB_SLAM3_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3)

# Check if the directory exists
if(NOT EXISTS ${ORB_SLAM3_DIR})
  message(FATAL_ERROR "ORB_SLAM3 directory not found at ${ORB_SLAM3_DIR}. Please update the ORB_SLAM3_DIR path.")
endif()

# Include ALL the necessary directories from ORB-SLAM3
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus     # Sophus library
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2      # DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/DBoW2 # Nested DBoW2 folder
  ${ORB_SLAM3_DIR}/Thirdparty/g2o        # g2o
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${catkin_LIBRARIES}
  -lboost_serialization
  -lcrypto
)

catkin_package(
  CATKIN_DEPENDS roscpp tf sensor_msgs image_transport cv_bridge
  LIBRARIES orb_slam3_ros
)

# Build the RGB-D version
add_executable(orb_slam3_ros_rgbd
  src/orb_slam3_ros_rgbd.cc
)

target_link_libraries(orb_slam3_ros_rgbd
  ${LIBS}
)

# Build the RGB-D version with YOLOv8 integration
add_executable(orb_slam3_ros_yolo_rgbd
  src/orb_slam3_ros_yolo_rgbd.cc
)

target_link_libraries(orb_slam3_ros_yolo_rgbd
  ${LIBS}
)

# Install targets
install(TARGETS orb_slam3_ros_rgbd orb_slam3_ros_yolo_rgbd
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install vocabulary and config files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/vocabulary
  FILES_MATCHING PATTERN "*"
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*"
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)