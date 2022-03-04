#!/usr/bin/env python

import math
import rospy
from gazebo_msgs.msg import LinkState, LinkStates, ModelStates, ModelState
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros



pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)


def callback(data):
    index = data.name.index("jackal::kinova_arm_right_finger_dist_link")
    eef_pose = data.pose[index]
    # q_rotation = quaternion_from_euler(0, 0, 3.14/2)
    # q = [eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z, eef_pose.orientation.w]
    # q_result = quaternion_multiply(q_rotation, q)
    # eef_pose.orientation.x = q_result[0]
    # eef_pose.orientation.y = q_result[1]
    # eef_pose.orientation.z = q_result[2]
    # eef_pose.orientation.w = q_result[3]
    # eef_pose.position.x = 1
    # eef_pose.position.y = 1
    # eef_pose.position.z = 1
    # pick_orientation = list(euler_from_quaternion([eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z, eef_pose.orientation.w]))
    # pick_orientation[1] += 3.14/2
    # pick_orientation = quaternion_from_euler(*pick_orientation)
    # # eef_pose.position.x -= 0.09
    # # eef_pose.position.y  += 0.1
    # # eef_pose.position.z  += 0.0
    # eef_pose.orientation.x = pick_orientation[0]
    # eef_pose.orientation.y = pick_orientation[1]
    # eef_pose.orientation.z = pick_orientation[2]
    # eef_pose.orientation.w = pick_orientation[3]
    # eef_pose.orientation.x += 3.14
    model_pose = eef_pose
    model_state = ModelState()
    model_state.model_name = "tomato_soup_can_textured"
    model_state.pose = model_pose
    model_state.reference_frame = "world"
    pub.publish(model_state)

def main():
    rospy.init_node('item_to_gripper', anonymous=True)
    # tf_buf = tf2_ros.Buffer()
    # tf_listener = tf2_ros.TransformListener(tf_buf)
    # src_pt = PoseStamped()
    # message = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    # src_pt.header.frame_id = "map"
    # src_pt.pose = message.pose[message.name.index("tomato_soup_can_textured")]
    # target_pt = tf_buf.transform(src_pt, "kinova_arm_base_link")
    # target_pt = tf_buf.transform(target_pt, "kinova_arm_end_effector_link")
    # # _end_effector_link
    # print(target_pt)

    # target_pt = tf_buf.transform(src_pt, "kinova_arm_base_link")
    # target_pt = tf_buf.transform(target_pt, "map")
    # rospy.init_node('item_to_gripper', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback)
    rospy.spin()


if __name__ == "__main__":
    main()