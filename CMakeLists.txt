cmake_minimum_required(VERSION 3.0.2)
project(kinova_moveit)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  gazebo
  message_generation
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)