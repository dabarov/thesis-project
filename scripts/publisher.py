#!/usr/bin/env python
import rospy
import std_msgs.msg


def main():
    rospy.init_node('publisher', anonymous=True)
    pub = rospy.Publisher('/int_msg', std_msgs.msg.String, queue_size=10)
    rate = rospy.Rate(10)
    pub.publish("Message")
    


if __name__ == "__main__":
    main() 
