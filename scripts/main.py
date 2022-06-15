#!/usr/bin/env python
import moveit_commander
import sys
import rospy


def main():
    rospy.init_node("moveit_dev")
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    arm = robot.get_group("manipulator")
    print(arm.get_current_joint_values())


if __name__ == "__main__":
    main()
