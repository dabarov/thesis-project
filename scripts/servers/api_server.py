#!/usr/bin/env python

import sys
import rospy
import tf2_ros
import tf2_geometry_msgs
import moveit_commander

from kinova_moveit.srv import Act
from kinova_moveit.msg import TargetObjectPcl
from geometry_msgs.msg import Point, PoseStamped

MOVE_HOME = "MOVE_HOME"
GRASP = "GRASP"
COMMANDS = {MOVE_HOME, GRASP}

# GRIPPER = "kinova_arm_right_finger_bottom_joint"
GRIPPER_GROUP = "gripper"
ARM = "manipulator"

HOME_STATE = [-4.121431299531221, 1.6010068431269513, 1.0524811740617386, 5.280673400664791, -4.870985093466171, -0.1311371491618747]
PICK_ORIENTATION = [-0.707, 0, 0, 0.707]

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = robot.get_group(ARM)
# gripper = robot.get_joint(GRIPPER)
gripper = robot.get_group(GRIPPER_GROUP)

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


def reach_named_position(group, target):
    group.set_named_target(target)
    group.execute(group.plan(), wait=True)


def get_bounding_box_size(boundingBoxAA, boundingBoxBB):
    size = [0, 0, 0]
    size[0] = abs(boundingBoxAA.x - boundingBoxBB.x)
    size[1] = abs(boundingBoxAA.y - boundingBoxBB.y)
    size[2] = abs(boundingBoxAA.z - boundingBoxBB.z)
    return size


def grasp():
    reach_named_position(gripper, "opened")
    params = rospy.wait_for_message("/kinova_moveit/pcl_params", TargetObjectPcl, timeout=10)
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
    tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform('base_link',
                                           'camera_depth_optical_frame',
                                           rospy.Duration(0.0),
                                           rospy.Duration(1.0))
    bouding_box_aa = PoseStamped()
    bouding_box_aa.header.frame_id = 'camera_depth_optical_frame'
    bouding_box_aa.pose.position.x = params.boundingBoxAA.x
    bouding_box_aa.pose.position.y = params.boundingBoxAA.y
    bouding_box_aa.pose.position.z = params.boundingBoxAA.z
    pose_transformed_aa = tf2_geometry_msgs.do_transform_pose(bouding_box_aa, transform)

    bouding_box_bb = PoseStamped()
    bouding_box_bb.header.frame_id = 'camera_depth_optical_frame'
    bouding_box_bb.pose.position.x = params.boundingBoxBB.x
    bouding_box_bb.pose.position.y = params.boundingBoxBB.y
    bouding_box_bb.pose.position.z = params.boundingBoxBB.z
    pose_transformed_bb = tf2_geometry_msgs.do_transform_pose(bouding_box_bb, transform)

    size = get_bounding_box_size(pose_transformed_aa.pose.position, pose_transformed_bb.pose.position)

    centroid_pose = PoseStamped()
    centroid_pose.header.frame_id = 'camera_depth_optical_frame'
    centroid_pose.pose.position.x = params.centroid.x
    centroid_pose.pose.position.y = params.centroid.y
    centroid_pose.pose.position.z = params.centroid.z
    pose_transformed = tf2_geometry_msgs.do_transform_pose(centroid_pose, transform)
    pose_transformed.pose.orientation.x = PICK_ORIENTATION[0]
    pose_transformed.pose.orientation.y = PICK_ORIENTATION[1]
    pose_transformed.pose.orientation.z = PICK_ORIENTATION[2]
    pose_transformed.pose.orientation.w = PICK_ORIENTATION[3]
    if reach_pose(arm=arm, pose=pose_transformed):
        box_pose = pose_transformed
        box_pose.pose.orientation.x = 0
        box_pose.pose.orientation.y = 0
        box_pose.pose.orientation.z = 0
        box_pose.pose.orientation.w = 1
        box_pose.header.frame_id = robot.get_planning_frame()
        scene.add_box("target", box_pose, tuple(size))


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
