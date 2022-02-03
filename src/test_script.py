#!/usr/bin/env python

import sys
import math
import rospy
import moveit_commander

from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from tf.transformations import quaternion_from_euler


FRAME_ID = "base_link"
SCENE = moveit_commander.PlanningSceneInterface()
X, Y, Z, W = 0, 1, 2, 3
OPEN = 0.9
CLOSE = 0.35
OBJECT_POSITIONS = {"box1": [0.3, 0.3, 0.15]}
PICK_ORIENTATION_EULER = [-math.pi / 2, 0, 0]
PLACE_ORIENTATION_EULER = [0, -math.pi / 2, 0]


def init_rosnode():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("gen3_lite_moveit_test")
    rospy.sleep(2)


def get_model_positions():
    message = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    return message.pose[1].position


def create_collision_object(id, dimensions, pose):
    object = CollisionObject()
    object.id = id
    object.header.frame_id = FRAME_ID

    solid = SolidPrimitive()
    solid.type = solid.BOX
    solid.dimensions = dimensions
    object.primitives = [solid]

    object_pose = Pose()
    object_pose.position.x = pose[X]
    object_pose.position.y = pose[Y]
    object_pose.position.z = pose[Z]

    object.primitive_poses = [object_pose]
    object.operation = object.ADD
    return object


def add_collision_objects():
    upper_limit = create_collision_object(id="upper_limit",
                                          dimensions=[1, 1, 0.01],
                                          pose=[0, 0.3, 0.65])
    back_limit = create_collision_object(id="back_limit",
                                         dimensions=[1, 0.01, 0.65],
                                         pose=[0, -0.2, 0.325])
    lower_limit = create_collision_object(id="lower_limit",
                                          dimensions=[10, 10, 0.2],
                                          pose=[0, 0, -0.1])
    box1 = create_collision_object(id="box1",
                                   dimensions=[0.05, 0.05, 0.2],
                                   pose=[0.3, 0.3, 0.1])

    SCENE.add_object(upper_limit)
    SCENE.add_object(lower_limit)
    SCENE.add_object(back_limit)
    SCENE.add_object(box1)


def reach_named_position(arm, target):
    arm.set_named_target(target)
    return arm.execute(arm.plan(), wait=True)


def reach_pose(arm, pose, tolerance=0.01):
    arm.set_pose_target(pose)
    arm.set_goal_position_tolerance(tolerance)
    return arm.go(wait=True)


def open_gripper(gripper):
    return gripper.move(gripper.max_bound() * OPEN, True)


def close_gripper(gripper):
    gripper.move(gripper.max_bound() * CLOSE, True)


def main():
    init_rosnode()
    add_collision_objects()
    arm = moveit_commander.MoveGroupCommander("arm", ns=rospy.get_namespace())
    robot = moveit_commander.RobotCommander("robot_description")
    gripper = robot.get_joint("right_finger_bottom_joint")
    arm.set_num_planning_attempts(5)
    # PICK
    pose = Pose()
    pose.position.x = OBJECT_POSITIONS["box1"][X]
    pose.position.y = OBJECT_POSITIONS["box1"][Y] - 0.1
    pose.position.z = OBJECT_POSITIONS["box1"][Z]
    orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
    pose.orientation.x = orientation[X]
    pose.orientation.y = orientation[Y]
    pose.orientation.z = orientation[Z]
    pose.orientation.w = orientation[W]

    reach_pose(arm, pose)
    open_gripper(gripper=gripper)
    pose.position.y += 0.1
    reach_pose(arm, pose)
    SCENE.remove_world_object("box1")
    close_gripper(gripper=gripper)

    # PLACE
    pose.position.z += 0.1
    reach_pose(arm, pose)
    pose.position.x = 0.1
    reach_pose(arm, pose)
    pose.position.z -= 0.1
    reach_pose(arm, pose)
    open_gripper(gripper=gripper)
    position = get_model_positions()
    box1 = create_collision_object(id="box1",
                                   dimensions=[0.05, 0.05, 0.2],
                                   pose=[position.x, position.y, position.z])
    SCENE.add_object(box1)
    reach_named_position(arm=arm, target="home")


if __name__ == "__main__":
    main()
