cmake_minimum_required(VERSION 3.0.2)
project(semantic_mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  pcl_ros
  nav_msgs
  message_generation
  yolov8_ros
)

find_package(OpenCV REQUIRED)

# Generate services
add_service_files(
  FILES
  QueryObject.srv
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  visualization_msgs
  tf 
  tf2_ros 
  tf2_geometry_msgs 
  cv_bridge 
  pcl_ros 
  nav_msgs
  message_runtime
  yolov8_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/semantic_mapper_node.py
  scripts/query_client.py
  scripts/semantic_navigator.py
  scripts/check_db.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)