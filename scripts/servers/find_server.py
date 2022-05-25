#!/usr/bin/env python

import os
import rospy
import rosnode
import roslaunch
from Queue import Queue
from kinova_moveit.srv import Find


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


def start_package(req, queue, target_q):
    if req.command == 'start':
        set_queue_item(queue)
        target_q.put(req)
        return 1
    reset_queue_item(queue)
    return -1


if __name__ == '__main__':
    queue = Queue(1)
    target_q = Queue(1)
    queue.put(False)
    package_launched = False
    rospy.init_node('moveit_find')
    start_package_service = rospy.Service('kinova_moveit/find', Find, lambda req: start_package(req, queue, target_q))
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    while not rospy.is_shutdown():
        if read_from_queue(queue) and not package_launched:
            target = target_q.get()
            node = roslaunch.core.Node(package="kinova_moveit", node_type="publish_pcl_params", output="screen")
            launch.launch(node)
            package_launched = True
        if not read_from_queue(queue) and package_launched:
            node_names = rosnode.get_node_names()
            node_to_kill = [node_name for node_name in node_names if node_name.startswith("/publish_pcl_params")]
            if len(node_to_kill):
                os.system("rosnode kill %s" % node_to_kill[0])
                rospy.loginfo("Released the object")
            package_launched = False
