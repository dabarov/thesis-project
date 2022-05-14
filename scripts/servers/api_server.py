#!/usr/bin/env python

import sys
import rospy
import tf2_ros
import tf2_geometry_msgs
import moveit_commander

from kinova_moveit.srv import Act
from geometry_msgs.msg import Point, PoseStamped


MOVE_HOME = "MOVE_HOME"
GRASP = "GRASP"
COMMANDS ={MOVE_HOME, GRASP}

GRIPPER = "kinova_arm_right_finger_bottom_joint"
ARM = "arm"

HOME_STATE = [-1.4784168770355182, -1.33056374371529, -0.677311965393744, 1.4564286586729214, -2.2202823107249765, -1.636241076589057]

robot = moveit_commander.RobotCommander()
arm = robot.get_group(ARM)
gripper = robot.get_joint(GRIPPER)

arm.set_planner_id("RRTConnect")
arm.set_num_planning_attempts(30)
arm.set_planning_time(30)


def init():
    rospy.init_node("kinova_moveit_api")
    moveit_commander.roscpp_initialize(sys.argv)


def move_home():
    while not arm.go(HOME_STATE):
        continue


def reach_pose(arm, pose, tolerance=0.00001):
    arm.set_pose_target(pose)
    arm.set_goal_position_tolerance(tolerance)
    return arm.go(wait=True)


def grasp():
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
    tf2_ros.TransformListener(tf_buffer)
    centroid = rospy.wait_for_message("/target_centroid", Point, timeout=10)
    transform = tf_buffer.lookup_transform('base_link',
                                           'camera_depth_optical_frame',
                                           rospy.Duration(0.0),
                                           rospy.Duration(1.0))
    centroid_pose = PoseStamped()
    centroid_pose.header.frame_id = 'camera_depth_optical_frame'
    centroid_pose.pose.position.x = centroid.x
    centroid_pose.pose.position.y = centroid.y
    centroid_pose.pose.position.z = centroid.z
    pose_transformed = tf2_geometry_msgs.do_transform_pose(centroid_pose, transform)
    pose_transformed.pose.orientation.x = -0.707
    pose_transformed.pose.orientation.y = 0
    pose_transformed.pose.orientation.z = 0
    pose_transformed.pose.orientation.w = 0.707
    reach_pose(arm=arm, pose=pose_transformed)



def handle(msg):
    if msg.command == MOVE_HOME:
        move_home()
        return "Move command completed"
    if msg.command == GRASP:
        grasp()
        return "Grasp command completed"
    return "Command %s is not available" % msg.command


def main():
    init()
    rospy.Service("kinova_moveit/api", Act, handle)
    rospy.loginfo("Enter one of the available commands: MOVE_HOME, GRASP")
    rospy.spin()



if __name__ == "__main__":
    main()
