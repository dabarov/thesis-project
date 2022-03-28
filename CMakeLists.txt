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
  geometry_msgs
)

add_service_files(DIRECTORY srv FILES TargetOffsetPose.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/box_to_hand.py
  scripts/dev.py
  scripts/little_actions.py
  scripts/move_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
