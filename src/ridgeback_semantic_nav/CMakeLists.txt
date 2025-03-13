cmake_minimum_required(VERSION 3.0.2)
project(ridgeback_semantic_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  move_base
  tf2_ros
  nav_msgs
  rviz
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  rospy
  move_base
  tf2_ros
  nav_msgs
  rviz
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python scripts installation
catkin_install_python(PROGRAMS
  scripts/navigate_to_object.py
  scripts/enhanced_semantic_navigator.py
  scripts/debug_navigation.py
  scripts/goto_object.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install configuration files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install RViz files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)