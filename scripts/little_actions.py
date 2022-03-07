#!/usr/bin/env python

import numpy
import rospy
import moveit_commander
import gazebo_msgs.msg
import geometry_msgs.msg
import tf.transformations 


def get_rotation_between(vec1, vec2):
    vec1_norm = tf.transformations.unit_vector(vec1)
    vec2_norm = tf.transformations.unit_vector(vec2)

    if numpy.allclose(numpy.array(vec1_norm), numpy.negative(numpy.array(vec2_norm))):
        return "dude"
    half = tf.transformations.unit_vector(vec1_norm + vec2_norm)
    x, y, z = numpy.cross(vec1_norm, half)
    w = numpy.dot(vec1_norm, half)
    return geometry_msgs.msg.Quaternion(x,y,z,w)



def quaternion_actions():
    eef_quat_1 = [0.00415399859043, 9.88038604144e-05, 0.00351505632348, 0.999985189347]
    eef_quat_2_inv = [-0.346786657004, 0.620200194586, 0.352310101367, -0.609071691703]
    print(tf.transformations.quaternion_multiply(eef_quat_1, eef_quat_2_inv))

    object_quat_1 = [0.151260809777, 0.69080153179, 0.156075159078, 0.689604202294]
    object_quat_2_inv = [-0.298247524098, 0.954448115937, 0.00214175029943, 0.00852181079548]
    print(tf.transformations.quaternion_multiply(object_quat_1, object_quat_2_inv))

    eef_position_1 = [0.0891739623886, -0.0117580601336, 1.17072632146]
    object_position_1 = [0.0895654289206, -0.0646766844511, 1.28544959601]
    offset_vector_1 = [eef_position_1[i] - object_position_1[i] for i in range(3)]
    print("offset_vector_1:", offset_vector_1)
    
    eef_position_2 = [0.426675657359, 0.0682746407914, 0.74728487455]
    object_position_2 = [0.530173315429, 0.140710697359, 0.74539639544]
    offset_vector_2 = [eef_position_2[i] - object_position_2[i] for i in range(3)]
    print("offset_vector_2:", offset_vector_2)

    rotation_quat = tf.transformations.quaternion_multiply(object_quat_1, object_quat_2_inv)
    rotation_quat_conj = tf.transformations.quaternion_conjugate(rotation_quat)
    rotated_offset_vector = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(rotation_quat, [offset_vector_1[0], offset_vector_1[1], offset_vector_1[2], 0]), 
                                                                    rotation_quat_conj)
    print("rotated_offset:", rotated_offset_vector[:3])

    print(get_rotation_between(offset_vector_1, offset_vector_2))
    


def robot_actions():
    robot = moveit_commander.RobotCommander()
    gripper_joint_name = "kinova_arm_right_finger_bottom_joint"
    gripper = robot.get_joint(gripper_joint_name)
    gripper.move(gripper.max_bound() * 0.9, True)


def main():
    rospy.init_node('little_actions', anonymous=True)
    link_states = rospy.wait_for_message("/gazebo/link_states", gazebo_msgs.msg.LinkStates)
    model_states = rospy.wait_for_message("/gazebo/model_states", gazebo_msgs.msg.ModelStates)

    eef_link_name = "jackal::kinova_arm_end_effector_link"
    eef_link_state_index = link_states.name.index(eef_link_name)
    eef_pose = link_states.pose[eef_link_state_index]
    eef_orientation_euler = tf.transformations.euler_from_quaternion([eef_pose.orientation.x, 
                                                                     eef_pose.orientation.y,
                                                                     eef_pose.orientation.z,
                                                                     eef_pose.orientation.w])
    print("EEF POSE:")
    print(eef_pose)
    print("IN ANGLES:")
    print(eef_orientation_euler)


    object_name = "tomato_soup_can_textured"
    object_state_index = model_states.name.index(object_name)
    object_pose = model_states.pose[object_state_index]
    object_orientation_euler = tf.transformations.euler_from_quaternion([object_pose.orientation.x, 
                                                                     object_pose.orientation.y,
                                                                     object_pose.orientation.z,
                                                                     object_pose.orientation.w])
    print("OBJECT POSE:")
    print(object_pose)
    print("IN ANGLES:")
    print(object_orientation_euler)


if __name__ == "__main__":
    # main()
    # robot_actions()
    quaternion_actions()