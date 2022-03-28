#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from kinova_moveit.srv import TargetOffsetPose


def init():
    rospy.init_node("moveit_dev")
    moveit_commander.roscpp_initialize(sys.argv)


def handle(req):
    rospy.loginfo("HEY THERE")
    rospy.loginfo(req)
    return 1


def main():
    # Literals
    gripper_joint_name = "kinova_arm_right_finger_bottom_joint"
    arm_group_name = "arm"

    # Move groups
    robot = moveit_commander.RobotCommander()
    arm = robot.get_group(arm_group_name)
    gripper = robot.get_joint(gripper_joint_name)

    # Arm configs
    arm.get_current_pose()
    arm.set_pose_reference_frame("base_link")
    arm.set_planner_id("RRTConnect")
    arm.set_num_planning_attempts(30)
    arm.set_planning_time(30)

    # Offset
    rospy.Service("kinova_moveit/move", TargetOffsetPose, handle)
    rospy.loginfo("Ready to move to target")
    rospy.spin()


if __name__ == "__main__":
    init()
    main()
