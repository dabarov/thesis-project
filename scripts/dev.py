#!/usr/bin/env python
import moveit_commander
import sys
import rospy
import math
import tf.transformations
import gazebo_msgs.msg
import geometry_msgs.msg


def move_gripper(gripper, position):
    return gripper.move(gripper.max_bound() * position, True)


def reach_pose(arm, pose, tolerance=0.00001):
    arm.set_pose_target(pose)
    arm.set_goal_position_tolerance(tolerance)
    return arm.go(wait=True)


def main():
     # Init roscpp node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_dev")

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

    # Define literals
    pick_orientation = tf.transformations.quaternion_from_euler(-math.pi / 2, 0, 0)
    place_orientation = tf.transformations.quaternion_from_euler(-math.pi / 2, 0, math.pi / 2)
    open_position = 0.9
    close_position = 0.6

    # Models and link messages
    link_states = rospy.wait_for_message("/gazebo/link_states", gazebo_msgs.msg.LinkStates)
    model_states = rospy.wait_for_message("/gazebo/model_states", gazebo_msgs.msg.ModelStates) 

    # EEF link pose in gazebo
    eef_link_name = "jackal::kinova_arm_end_effector_link"
    eef_link_state_index = link_states.name.index(eef_link_name)
    eef_link_pose = link_states.pose[eef_link_state_index]

    # EEF link in moveit
    eef_moveit_pose = arm.get_current_pose()
    
    # Offset position
    offset_pose = geometry_msgs.msg.Pose()
    offset_pose.position.x = eef_moveit_pose.pose.position.x - eef_link_pose.position.x
    offset_pose.position.y = eef_moveit_pose.pose.position.y - eef_link_pose.position.y
    offset_pose.position.z = eef_moveit_pose.pose.position.z - eef_link_pose.position.z

    # Target position to grasp
    target_name = "tomato_soup_can_textured"
    target_state_index = model_states.name.index(target_name)
    target_pose_map = model_states.pose[target_state_index]
    
    # Target position with offset
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = target_pose_map.position.x + offset_pose.position.x
    target_pose.position.y = target_pose_map.position.y + offset_pose.position.y
    target_pose.position.z = target_pose_map.position.z + offset_pose.position.z

    # Add pick orientation to target pose
    target_pose.orientation.x = pick_orientation[0]
    target_pose.orientation.y = pick_orientation[1]
    target_pose.orientation.z = pick_orientation[2]
    target_pose.orientation.w = pick_orientation[3]
    # Add pregrasping offset 
    target_pose.position.y -= 0.2
    target_pose.position.z += 0.05

    print(reach_pose(arm=arm, pose=target_pose))
    print(move_gripper(gripper=gripper, position=open_position))

    target_pose.position.y += 0.15
    print(reach_pose(arm=arm, pose=target_pose))
    print(move_gripper(gripper=gripper, position=close_position))
    

if __name__ == "__main__":
    main()