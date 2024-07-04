cmake_minimum_required(VERSION 3.0.2)
project(HPHS)

find_package(catkin REQUIRED COMPONENTS
  rospy
  pcl_ros
  pcl_conversions
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/dtw.py
  scripts/utils.py
  scripts/explorer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

