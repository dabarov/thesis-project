cmake_minimum_required(VERSION 3.0.2)
project(kinova_moveit)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  gazebo
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES 
  TargetObjectPcl.msg
)

add_service_files(DIRECTORY srv FILES 
  Act.srv
  TargetOffsetPose.srv
  Grab.srv
  Release.srv  
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/box_to_hand.py
  scripts/dev.py
  scripts/little_actions.py
  scripts/servers/move_server.py
  scripts/servers/grab_server.py
  scripts/servers/api_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(client src/client.cpp)

add_executable(publish_pcl_params src/getPclParams.cpp)
target_link_libraries(publish_pcl_params ${catkin_LIBRARIES} ${PCL_LIBRARIES})

target_link_libraries(client ${catkin_LIBRARIES})

add_dependencies(client kinova_moveit_gencpp)
