#!/usr/bin/env python

import sys
import math
from unicodedata import name
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
CLOSE = 0.126
OBJECT_POSITIONS = {"target_1": [0.05, 0.35, 0.3],
                    "target_2": [0.15, 0.35, 0.3],
                    "target_3": [0.25, 0.35, 0.3],
                    "target_4": [0.05, 0.45, 0.3],
                    "target_5": [0.15, 0.45, 0.3],
                    "target_6": [0.25, 0.45, 0.3]}
PICK_ORIENTATION_EULER = [-math.pi / 2, 0, 0]
PLACE_ORIENTATION_EULER = [-math.pi / 2, 0, -math.pi / 2]


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
                                          pose=[0, 0.2, 0.65])
    back_limit = create_collision_object(id="back_limit",
                                         dimensions=[1, 0.01, 0.65],
                                         pose=[0, -0.3, 0.325])
    floor_limit = create_collision_object(id="floor_limit",
                                          dimensions=[10, 10, 0.2],
                                          pose=[0, 0, -0.1])
    table_1 = create_collision_object(id="table_1",
                                      dimensions=[0.3, 0.6, 0.2],
                                      pose=[0.45, 0.3, 0.1])
    table_2 = create_collision_object(id="table_2",
                                      dimensions=[0.3, 0.3, 0.2],
                                      pose=[0.15, 0.45, 0.1])
    target_1 = create_collision_object(id="target_1",
                                       dimensions=[0.02, 0.02, 0.2],
                                       pose=[0.05, 0.35, 0.3])
    target_2 = create_collision_object(id="target_2",
                                       dimensions=[0.02, 0.02, 0.2],
                                       pose=[0.15, 0.35, 0.3])
    target_3 = create_collision_object(id="target_3",
                                       dimensions=[0.02, 0.02, 0.2],
                                       pose=[0.25, 0.35, 0.3])
    target_4 = create_collision_object(id="target_4",
                                       dimensions=[0.02, 0.02, 0.2],
                                       pose=[0.05, 0.45, 0.3])
    target_5 = create_collision_object(id="target_5",
                                       dimensions=[0.02, 0.02, 0.2],
                                       pose=[0.15, 0.45, 0.3])
    target_6 = create_collision_object(id="target_6",
                                       dimensions=[0.02, 0.02, 0.2],
                                       pose=[0.25, 0.45, 0.3])
    SCENE.add_object(upper_limit)
    SCENE.add_object(floor_limit)
    SCENE.add_object(back_limit)
    SCENE.add_object(table_1)
    SCENE.add_object(table_2)
    SCENE.add_object(target_1)
    SCENE.add_object(target_2)
    SCENE.add_object(target_3)
    SCENE.add_object(target_4)
    SCENE.add_object(target_5)
    SCENE.add_object(target_6)


def reach_named_position(arm, target):
    arm.set_named_target(target)
    return arm.execute(arm.plan(), wait=True)


def reach_pose(arm, pose, tolerance=0.001):
    arm.set_pose_target(pose)
    arm.set_goal_position_tolerance(tolerance)
    return arm.go(wait=True)


def open_gripper(gripper):
    return gripper.move(gripper.max_bound() * OPEN, True)


def close_gripper(gripper):
    gripper.move(gripper.max_bound() * CLOSE, True)


def pick_object(name, arm, gripper):
    pose = Pose()
    pose.position.x = OBJECT_POSITIONS[name][X]
    pose.position.y = OBJECT_POSITIONS[name][Y] - 0.1
    pose.position.z = OBJECT_POSITIONS[name][Z]
    orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
    pose.orientation.x = orientation[X]
    pose.orientation.y = orientation[Y]
    pose.orientation.z = orientation[Z]
    pose.orientation.w = orientation[W]
    reach_pose(arm, pose)
    open_gripper(gripper=gripper)
    pose.position.y += 0.1
    reach_pose(arm, pose)
    close_gripper(gripper=gripper)
    # arm.attach_object(name)


def place_object(name, arm, gripper, row=1):
    rows = [0.35, 0.45]
    pose = Pose()
    pose.position.x = rows[row]
    pose.position.y = OBJECT_POSITIONS[name][X]
    pose.position.z = OBJECT_POSITIONS[name][Z]
    orientation = quaternion_from_euler(*PLACE_ORIENTATION_EULER)
    pose.orientation.x = orientation[X]
    pose.orientation.y = orientation[Y]
    pose.orientation.z = orientation[Z]
    pose.orientation.w = orientation[W]

    reach_pose(arm, pose)
    open_gripper(gripper=gripper)
    reach_pose(arm, pose)
    arm.detach_object(name)


def main():
    init_rosnode()
    arm = moveit_commander.MoveGroupCommander("arm", ns=rospy.get_namespace())
    robot = moveit_commander.RobotCommander("robot_description")
    gripper = robot.get_joint("right_finger_bottom_joint")
    arm.set_num_planning_attempts(45)
    SCENE.remove_world_object("upper_limit")
    SCENE.remove_world_object("back_limit")
    SCENE.remove_world_object("floor_limit")
    SCENE.remove_world_object("table_1")
    SCENE.remove_world_object("table_2")
    SCENE.remove_world_object("target_1")
    SCENE.remove_world_object("target_2")
    SCENE.remove_world_object("target_3")
    SCENE.remove_world_object("target_4")
    SCENE.remove_world_object("target_5")
    SCENE.remove_world_object("target_6")
    reach_named_position(arm=arm, target="home")
    add_collision_objects()
    # TARGET1
    # PICK
    pick_object(name="target_1", arm=arm, gripper=gripper)

    # # PLACE
    # place_object(name="target_1", arm=arm, gripper=gripper)

    # # TARGET3
    # # PICK
    # pick_object(name="target_3", arm=arm, gripper=gripper)

    # # PLACE
    # place_object(name="target_3", arm=arm, gripper=gripper)

    # # TARGET2
    # # PICK
    # pick_object(name="target_2", arm=arm, gripper=gripper)

    # # PLACE
    # place_object(name="target_2", arm=arm, gripper=gripper)

    # # TARGET4
    # # PICK
    # pick_object(name="target_4", arm=arm, gripper=gripper)

    # # PLACE
    # place_object(name="target_4", arm=arm, gripper=gripper, row=0)

    # # TARGET5
    # # PICK
    # pick_object(name="target_5", arm=arm, gripper=gripper)

    # # PLACE
    # place_object(name="target_5", arm=arm, gripper=gripper, row=0)

    # # TARGET6
    # # PICK
    # pick_object(name="target_6", arm=arm, gripper=gripper)

    # # PLACE
    # place_object(name="target_6", arm=arm, gripper=gripper, row=0)

    # reach_named_position(arm=arm, target="home")

if __name__ == "__main__":
    main()
