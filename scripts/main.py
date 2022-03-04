#!/usr/bin/env python

from email import header
import sys
import math
import rospy
import moveit_commander
import tf2_ros
from tf2_geometry_msgs import PointStamped
from gazebo_msgs.msg import ModelState, LinkStates

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped
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
    pose.position.x = -0.0659186383706
    pose.position.y = -0.0348770169619
    pose.position.z = 0.825501871609
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

pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

def callback(data, args):
    target_pt = args
    tf_buf = tf2_ros.Buffer()
    rate = rospy.Rate(150)

   

    model_state = ModelState()
    model_state.model_name = "tomato_soup_can_textured"
    model_state.pose = tf_buf.pose
    model_state.reference_frame = "world"

def main():
    # Init roscpp node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("gen3_lite_moveit")

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
    arm.set_planning_time(30)

    # Scene setup

    # Define literals
    pick_orientation = quaternion_from_euler(-math.pi / 2, 0, 0)
    place_orientation = quaternion_from_euler(-math.pi / 2, 0, math.pi / 2)
    open_position = 0.9
    close_position = 0.43

    # look_at_table(arm, pick_orientation)
    # get_to_tomato_can(arm, pick_orientation)
    # move_gripper(gripper=gripper, position=open_position)
    # get_closer_to_tomato_can(arm, pick_orientation)
    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    src_pt = PoseStamped()
    message = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    src_pt.header.frame_id = "base_link"
    src_pt.pose = message.pose[message.name.index("tomato_soup_can_textured")]
    print(src_pt)
    rate = rospy.Rate(150)
    while True:
        try:
            target_pt = tf_buf.transform(src_pt, "chassis_link")
            break
        except:
            rate.sleep()
    while True:
        try:
            target_pt = tf_buf.transform(target_pt, "mid_mount")
            break
        except:
            rate.sleep()
    while True:
        try:
            target_pt = tf_buf.transform(target_pt, "kinova_arm_base_link")
            break
        except:
            rate.sleep()
    while True:
        try:
            target_pt = tf_buf.transform(target_pt, "kinova_arm_end_effector_link")
            break
        except:
            rate.sleep()
    while True:
        try:
            target_pt = tf_buf.transform(target_pt, "kinova_arm_base_link")
            break
        except:
            rate.sleep()
    while True:
        try:
            target_pt = tf_buf.transform(target_pt, "mid_mount")
            break
        except:
            rate.sleep()
    while True:
        try:
            target_pt = tf_buf.transform(target_pt, "chassis_link")
            break
        except:
            rate.sleep()
    while True:
        try:
            target_pt = tf_buf.transform(target_pt, "base_link")
            break
        except:
            rate.sleep()
    print(target_pt)
    pub.publish(model_state)

    
    # move_gripper(gripper=gripper, position=close_position)
    # lift_tomato_can(arm, pick_orientation)
    # move_tomato_can(arm, place_orientation)
    # print(arm.get_current_pose())
    # print(gripper.value())



if __name__ == "__main__":
    main()