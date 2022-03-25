#!/usr/bin/env python
import rospy
import std_msgs.msg

def callback(data):
    rospy.loginfo(data)
    rospy.signal_shutdown("Dead")

def main():
    rospy.init_node('waiter', anonymous=True)
    rospy.Subscriber("/int_msg", std_msgs.msg.String, callback)
    rospy.spin()    


if __name__ == "__main__":
    main() 
