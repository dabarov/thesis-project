#!/usr/bin/env python

import numpy
import sys
import rospy
from gazebo_msgs.msg import LinkStates, ModelStates, ModelState
from geometry_msgs.msg import Pose
import tf.transformations


pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)


def callback(data, args):
    initial_eef_pose = args[0]
    initial_object_pose = args[1]
    object_name = args[2]

    initial_eef_quat = [initial_eef_pose.orientation.x,
                    initial_eef_pose.orientation.y,
                    initial_eef_pose.orientation.z,
                    initial_eef_pose.orientation.w] 
    initial_eef_quat_conj = tf.transformations.quaternion_conjugate(initial_eef_quat)
    initial_object_quat = [initial_object_pose.orientation.x,
                    initial_object_pose.orientation.y,
                    initial_object_pose.orientation.z,
                    initial_object_pose.orientation.w]                
    index = data.name.index("jackal::kinova_arm_link_6")
    current_eef_pose = data.pose[index]
    current_quat = [current_eef_pose.orientation.x,
                    current_eef_pose.orientation.y,
                    current_eef_pose.orientation.z,
                    current_eef_pose.orientation.w]
    rotation_quaternion = tf.transformations.quaternion_multiply(current_quat, initial_eef_quat_conj)
    roation_quaternion_matrix = tf.transformations.quaternion_matrix(rotation_quaternion)
    rotation_matrix = numpy.delete(numpy.delete(roation_quaternion_matrix, 3, 0), 3, 1)
    offset = Pose()
    offset = numpy.array([initial_object_pose.position.x - initial_eef_pose.position.x,
                          initial_object_pose.position.y - initial_eef_pose.position.y,
                          initial_object_pose.position.z - initial_eef_pose.position.z])

    new_object_quat = tf.transformations.quaternion_multiply(rotation_quaternion, initial_object_quat)
    new_offset = rotation_matrix.dot(offset)
    
    new_object_pose = Pose()
    new_object_pose.orientation.x = new_object_quat[0]
    new_object_pose.orientation.y = new_object_quat[1]
    new_object_pose.orientation.z = new_object_quat[2]
    new_object_pose.orientation.w = new_object_quat[3]

    new_object_pose.position.x = current_eef_pose.position.x + new_offset[0]
    new_object_pose.position.y = current_eef_pose.position.y + new_offset[1]
    new_object_pose.position.z = current_eef_pose.position.z + new_offset[2]

    new_object_state = ModelState()
    new_object_state.pose = new_object_pose
    new_object_state.model_name = object_name
    new_object_state.reference_frame = "world"
    pub.publish(new_object_state)


def main():
    rospy.init_node('item_to_gripper', anonymous=True)
    link_states = rospy.wait_for_message("/gazebo/link_states", LinkStates)
    eef_link_name = "jackal::kinova_arm_link_6"
    eef_link_state_index = link_states.name.index(eef_link_name)
    eef_pose = link_states.pose[eef_link_state_index]

    model_states = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    object_name = "mustard_bottle_textured"
    if len(sys.argv) > 1:
        object_name = sys.argv[1]
    rospy.loginfo(object_name)
    object_state_index = model_states.name.index(object_name)
    object_pose = model_states.pose[object_state_index]
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback, [eef_pose, object_pose, object_name])
    rospy.spin()


if __name__ == "__main__":
    main()