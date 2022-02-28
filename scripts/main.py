#!/usr/bin/env python

import sys
import math
import rospy
import moveit_commander

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


def move_gripper(gripper, position):
    return gripper.move(gripper.max_bound() * position, True)


def reach_pose(arm, pose, tolerance=0.00001):
    arm.set_pose_target(pose)
    arm.set_goal_position_tolerance(tolerance)
    return arm.go(wait=True)


def reach_named_position(arm, target):
    arm.set_named_target(target)
    arm.execute(arm.plan(), wait=True)


def look_at_table(arm, orientation):
    pose = Pose()
    pose.position.x = -0.276568463654
    pose.position.y = -0.0348770169619
    pose.position.z = 0.825501871609
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose(arm, pose)

    pose = Pose()
    pose.position.x = 0.1
    pose.position.y = -0.0348770169619
    pose.position.z = 0.825501871609
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose(arm, pose)

    pose = Pose()
    pose.position.x = -0.044485960504
    pose.position.y = 0.0189430342413
    pose.position.z = 0.389727671487
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose(arm, pose)

    pose = Pose()
    pose.position.x = -0.352207153425
    pose.position.y = 0.0463408132853
    pose.position.z = 0.387629905083
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose(arm, pose)


def get_to_tomato_can(arm, orientation):
    pose = Pose()
    pose.position.x = -0.0659186383706
    pose.position.y = 0.351257134702
    pose.position.z = 0.788471110169
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose(arm, pose)

    
def get_closer_to_tomato_can(arm, orientation):
    pose = Pose()
    pose.position.x = -0.0659186383706
    pose.position.y = 0.409357782211
    pose.position.z = 0.788471110169
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose(arm, pose)

def lift_tomato_can(arm, orientation):
    pose = Pose()
    pose.position.x = -0.0699186383706
    pose.position.y = 0.409357782211
    pose.position.z = 0.838471110169
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose(arm, pose)

def move_tomato_can(arm, orientation):
    pose = Pose()
    pose.position.x = -0.276568463654
    pose.position.y = -0.0348770169619
    pose.position.z = 0.825501871609
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose(arm, pose)


def main():
    # Init roscpp node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("kinova_moveit")

    # Robot and scene
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Arm and gripper
    gripper_joint_name = "kinova_arm_right_finger_bottom_joint"
    arm_group_name = "arm"
    arm = robot.get_group(arm_group_name)
    gripper = robot.get_joint(gripper_joint_name)

    # Planning configs
    arm.set_pose_reference_frame("base_link")
    arm.set_planner_id("RRTConnect")
    arm.set_num_planning_attempts(30)
    arm.set_planning_time(10)

    # Scene setup

    # Define literals
    pick_orientation = quaternion_from_euler(-math.pi / 2, 0, 0)
    place_orientation = quaternion_from_euler(-math.pi / 2, 0, -math.pi / 2)
    open_position = 0.9
    close_position = 0.43

    look_at_table(arm, pick_orientation)
    get_to_tomato_can(arm, pick_orientation)
    move_gripper(gripper=gripper, position=open_position)
    get_closer_to_tomato_can(arm, pick_orientation)
    move_gripper(gripper=gripper, position=close_position)
    lift_tomato_can(arm, pick_orientation)
    move_tomato_can(arm, place_orientation)
    print(arm.get_current_pose())
    print(gripper.value())



if __name__ == "__main__":
    main()