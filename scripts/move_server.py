#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from kinova_moveit.srv import TargetOffsetPose


def reach_pose(arm, pose, tolerance=0.00001):
    arm.set_pose_target(pose)
    arm.set_goal_position_tolerance(tolerance)
    return arm.go(wait=True)


def init():
    rospy.init_node("moveit_dev")
    moveit_commander.roscpp_initialize(sys.argv)


def handle(position):
    rospy.loginfo(position)
    current_pose = arm.get_current_pose().pose
    current_pose.position.x += position.x
    current_pose.position.y += position.y
    current_pose.position.z += position.z

    if reach_pose(arm=arm, pose=current_pose):
        return 1
    return -1


# Main
init()

# Literals
GRIPPER = "kinova_arm_right_finger_bottom_joint"
ARM = "arm"

# Move groups
robot = moveit_commander.RobotCommander()
arm = robot.get_group(ARM)
gripper = robot.get_joint(GRIPPER)

# Arm configs
arm.get_current_pose()
arm.set_pose_reference_frame("base_link")
arm.set_planner_id("RRTConnect")
arm.set_num_planning_attempts(30)
arm.set_planning_time(30)

rospy.Service("kinova_moveit/move", TargetOffsetPose, handle)
rospy.loginfo("Ready to move to target")
rospy.spin()
