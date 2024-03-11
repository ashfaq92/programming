#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


def talker():
    # initialize a node
    rospy.init_node('talker', anonymous=True)
    publisher_instance = rospy.Publisher('chatter_py_topic', String, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        publisher_instance.publish(hello_str)
        rate.sleep()
    

if __name__ == "__main__":
    talker()
   
