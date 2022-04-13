#include "ros/ros.h"
#include "kinova_moveit/TargetOffsetPose.h"
#include "kinova_moveit/Grab.h"
#include "kinova_moveit/Release.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinova_moveit_client");
  ros::NodeHandle n1;
  ros::NodeHandle n2;
  ros::NodeHandle n3;
  ros::ServiceClient move_client = n1.serviceClient<kinova_moveit::TargetOffsetPose>("/kinova_moveit/move");
  ros::ServiceClient grap_client = n2.serviceClient<kinova_moveit::Grab>("/kinova_moveit/grab");
  ros::ServiceClient release_client = n3.serviceClient<kinova_moveit::Release>("/kinova_moveit/release");

  // Move client
  kinova_moveit::TargetOffsetPose move_srv; 
  move_srv.request.close = 0.9;
  move_srv.request.offset.x = 0.0;
  move_srv.request.offset.y = 0.1;
  move_srv.request.offset.z = 0.0;
  move_client.call(move_srv);
  
  // Grab client
  kinova_moveit::Grab grab_srv; 
  grab_srv.request.targetName = "orange";
  grap_client.call(move_srv);
  return 0;
}