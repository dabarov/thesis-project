#!/usr/bin/env python

import roslaunch
import rospy
import sys
import moveit_commander
from kinova_moveit.srv import TargetOffsetPose


node = roslaunch.core.Node("kinova_moveit", "box_to_hand.py")
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)

rospy.loginfo("started")

while process.is_alive():
    print("HEY")
print("NO")




def move_gripper(gripper, position):
    return gripper.move(gripper.max_bound() * position, True)


def reach_pose(arm, pose, tolerance=0.00001):
    arm.set_pose_target(pose)
    arm.set_goal_position_tolerance(tolerance)
    return arm.go(wait=True)


def init():
    rospy.init_node("moveit_dev")
    moveit_commander.roscpp_initialize(sys.argv)


def handle(msg):
    return -1


# Main

rospy.Service("kinova_moveit/grab", TargetOffsetPose, handle)
rospy.loginfo("Grabbing")
rospy.spin()
