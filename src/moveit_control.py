#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose


def init_rosnode():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("gen3_lite_moveit")


def init_commanders(ns):
    arm = moveit_commander.MoveGroupCommander("arm", ns=ns)
    gripper = moveit_commander.MoveGroupCommander("gripper", ns=ns)
    return arm, gripper


def reach_named_position(arm, target):
    arm.set_named_target(target)
    return arm.execute(arm.plan(), wait=True)


def reach_pose(arm, target, tolerance=0.1):
    arm.set_pose_target(target)
    arm.set_goal_position_tolerance(tolerance)
    return arm.go(wait=True)


def main():
    init_rosnode()
    namespace = rospy.get_namespace()
    arm, _ = init_commanders(ns=namespace)
    reach_named_position(arm=arm, target="home")
    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    reach_pose(arm=arm, target=pose)


if __name__ == "__main__":
    main()
