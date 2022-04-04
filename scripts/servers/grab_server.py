#!/usr/bin/env python

import os
import rospy
import rosnode
import roslaunch
from Queue import Queue
from kinova_moveit.srv import Grab, Release


def set_queue_item(queue):
    queue.get()
    queue.put(True)


def reset_queue_item(queue):
    queue.get()
    queue.put(False)


def read_from_queue(queue):
    current_item = queue.get()
    queue.put(current_item)
    return current_item


def start_package(req, queue):
    set_queue_item(queue)
    return -1

def stop_package(req, queue):
    reset_queue_item(queue)



if __name__ == '__main__':
    queue = Queue(1)
    queue.put(False)
    package_launched = False
    rospy.init_node('moveit_grab')
    start_package_service = rospy.Service('kinova_moveit/grab', Grab, lambda req: start_package(req, queue))
    stop_package_service = rospy.Service('kinova_moveit/release', Release, lambda req: stop_package(req, queue))
    node = roslaunch.core.Node("kinova_moveit", "box_to_hand.py")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    while not rospy.is_shutdown():
        if read_from_queue(queue) and not package_launched:
            launch.launch(node)
            rospy.loginfo("NODE LAUNCHED")   # implement your custom launch method using the roslaunch API
            package_launched = True
        if not read_from_queue(queue) and package_launched:
            node_names = rosnode.get_node_names()
            node_to_kill = [node_name for node_name in node_names if node_name.startswith("/box_to_hand")]
            os.system("rosnode kill %s" % node_to_kill[0])  # implement your custom stop method using the roslaunch API
            package_launched = False

# Main
# rospy.init_node("grab_server")
# rospy.Service("kinova_moveit/grab", Grab, handle)

# node = roslaunch.core.Node("kinova_moveit", "box_to_hand.py")
# launch = roslaunch.scriptapi.ROSLaunch()
# launch.start()
# process = launch.launch(node)

# rospy.loginfo("started")

# while process.is_alive():
#     print("HEY")
# print("NO")
# rospy.spin()
