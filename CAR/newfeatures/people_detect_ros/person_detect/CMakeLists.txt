cmake_minimum_required(VERSION 2.8.3)
project(person_detect)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy cv_bridge sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/person_detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)