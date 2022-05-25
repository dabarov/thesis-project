#include "ros/ros.h"

#include "kinova_moveit/Act.h"
#include "kinova_moveit/Find.h"
#include "kinova_moveit/Grab.h"
#include "kinova_moveit/Release.h"
#include "kinova_moveit/TargetOffsetPose.h"

#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinova_moveit_client");

  ros::NodeHandle n1;
  ros::NodeHandle n2;
  ros::NodeHandle n3;
  ros::NodeHandle n4;
  ros::NodeHandle n5;

  ros::ServiceClient api_client = n1.serviceClient<kinova_moveit::Act>("/kinova_moveit/api");
  ros::ServiceClient grap_client = n2.serviceClient<kinova_moveit::Grab>("/kinova_moveit/grab");
  ros::ServiceClient find_client = n3.serviceClient<kinova_moveit::Find>("/kinova_moveit/find");
  ros::ServiceClient move_client = n4.serviceClient<kinova_moveit::TargetOffsetPose>("/kinova_moveit/move");
  ros::ServiceClient release_client = n5.serviceClient<kinova_moveit::Release>("/kinova_moveit/release");

  // move home
  kinova_moveit::Act api_action;
  api_action.request.command = "MOVE_HOME";
  api_client.call(api_action);

  // find object
  kinova_moveit::Find find_action;
  find_action.request.command = "start";
  find_client.call(find_action); 

  // approach object
  api_action.request.command = "GRASP";
  api_client.call(api_action);

  // grab object
  kinova_moveit::Grab grab_action;
  grab_action.request.targetName = "tomato_soup_can_textured";
  grap_client.call(grab_action);

  // move home
  api_action.request.command = "MOVE_HOME";
  api_client.call(api_action);

  return 0;
}