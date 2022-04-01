#!/usr/bin/env python

import roslaunch
import rospy
import os
node = roslaunch.core.Node("kinova_moveit", "box_to_hand.py")

os.system("rosnode kill box_to_hand_py_robot0_12251_6422409149559055913")
rospy.loginfo("stoped")