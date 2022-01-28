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
    robot = moveit_commander.RobotCommander("robot_description")
    return arm, robot


def reach_named_position(arm, target):
    arm.set_named_target(target)
    return arm.execute(arm.plan(), wait=True)


def reach_pose(arm, target, tolerance=0.01):
    arm.set_pose_target(target)
    arm.set_goal_position_tolerance(tolerance)
    return arm.go(wait=True)


def get_pose(arm):
    pose = arm.get_current_pose()
    return pose.pose


def open_gripper(gripper):
    gripper.move(gripper.max_bound() * 0.9, True)


def close_gripper(gripper, value):
    gripper.move(gripper.max_bound() * value, True)


# 0.2 1.0 0.778272
# -0,096837 0,572028 0,095835


def main():
    init_rosnode()
    namespace = rospy.get_namespace()
    arm, robot = init_commanders(ns=namespace)
    gripper = robot.get_joint("right_finger_bottom_joint")

    reach_named_position(arm=arm, target="home")

    # dummy for proper path
    pose = Pose()
    pose.position.x = 0.215483647276
    pose.position.y = 0.309998333761
    pose.position.z = 0.0474959065835
    pose.orientation.x = 0.193297518685
    pose.orientation.y = 0.683525381274
    pose.orientation.z = 0.68216296102
    pose.orientation.w = 0.173443988184
    reach_pose(arm=arm, target=pose)

    # infront of mustard bottle
    pose = Pose()
    pose.position.x = 0.1585
    pose.position.y = 0.420991
    pose.position.z = 0.05257
    pose.orientation.x = 0.1893
    pose.orientation.y = 0.6843
    pose.orientation.z = 0.6815
    pose.orientation.w = 0.177
    reach_pose(arm=arm, target=pose)

    open_gripper(gripper=gripper)

    pose = get_pose(arm)
    pose.position.x += 0.05
    pose.position.y += 0.05
    reach_pose(arm=arm, target=pose)

    for i in [0.5, 0.6, 0.5, 0.5, 0.6, 0.5, 0.5, 0.6, 0.5]:
        try:
            close_gripper(gripper=gripper, value=i)
            break
        except:
            pass

    reach_named_position(arm=arm, target="home")

    open_gripper(gripper=gripper)


if __name__ == "__main__":
    main()
