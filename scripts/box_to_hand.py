#!/usr/bin/env python

import numpy
import rospy
from gazebo_msgs.msg import LinkState, LinkStates, ModelStates, ModelState
from geometry_msgs.msg import Pose, PoseStamped
import tf.transformations



pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)


# def callback(data):
#     index = data.name.index("jackal::kinova_arm_right_finger_dist_link")
#     eef_pose = data.pose[index]
#     model_pose = eef_pose
#     model_state = ModelState()
#     model_state.model_name = "tomato_soup_can_textured"
#     model_state.pose = model_pose
#     model_state.reference_frame = "world"
#     pub.publish(model_state)
def callback(data, args):
    initial_eef_pose = args
    initial_quat = [initial_eef_pose.orientation.x,
                    initial_eef_pose.orientation.y,
                    initial_eef_pose.orientation.z,
                    initial_eef_pose.orientation.w] 
    index = data.name.index("jackal::kinova_arm_end_effector_link")
    current_eef_pose = data.pose[index]
    current_quat_inv = [current_eef_pose.orientation.x,
                    current_eef_pose.orientation.y,
                    current_eef_pose.orientation.z,
                    -current_eef_pose.orientation.w]
    rotation_quaternion = tf.transformations.quaternion_multiply(initial_quat, current_quat_inv)
    roation_quaternion_matrix = tf.transformations.quaternion_matrix(rotation_quaternion)
    rotation_matrix = numpy.delete(numpy.delete(roation_quaternion_matrix, 3, 0), 3, 1)
    print(rotation_matrix)

def main():
    rospy.init_node('item_to_gripper', anonymous=True)
    link_states = rospy.wait_for_message("/gazebo/link_states", LinkStates)
    eef_link_name = "jackal::kinova_arm_end_effector_link"
    eef_link_state_index = link_states.name.index(eef_link_name)
    eef_pose = link_states.pose[eef_link_state_index]

    model_states = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    object_name = "tomato_soup_can_textured"
    object_state_index = model_states.name.index(object_name)
    object_pose = model_states.pose[object_state_index]
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback, [eef_pose, object_pose])
    rospy.spin()


if __name__ == "__main__":
    main()