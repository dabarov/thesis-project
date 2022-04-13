#include "ros/ros.h"
#include "kinova_moveit/TargetOffsetPose.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinova_moveit_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<kinova_moveit::TargetOffsetPose>("/kinova_moveit/move");
  kinova_moveit::TargetOffsetPose srv; 
  srv.request.close = 0.9;
  srv.request.offset.x = 0.0;
  srv.request.offset.y = 0.1;
  srv.request.offset.z = 0.0;
  client.call(srv);
  return 0;
}