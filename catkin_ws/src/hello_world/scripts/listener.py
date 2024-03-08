#!/usr/bin/env python3


import rospy
from std_msgs.msg import String


def msg_printer(msg):
    # rospy.loginfo(rospy.get_caller_id(), "I heard", msg.data)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)

def listener():
    rospy.init_node("listener", anonymous=True)
    
    rospy.Subscriber("chatter_py_topic", String, msg_printer)

    rospy.spin()

if __name__ == "__main__":
    listener()