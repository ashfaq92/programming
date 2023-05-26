#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


class ControlTrutleBot():
    def __init__(self):
        rospy.init_node('ControlTurtleBot', anonymous=False)

        # message to screen 
        rospy.loginfo("Press Ctrl+c to stop the TurtleBot")

        rospy.on_shutdown(self.shutdown)
        
        # self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)


        rate = rospy.Rate(10)

        move_cmd = Twist()

        move_cmd.linear.x = 0.3

        move_cmd.angular.z = 0.5

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)
            rate.sleep()

    def shutdown(self):
        rospy.loginfo('stopping turtlebot')
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        ControlTrutleBot()
    except:
        rospy.loginfo('end of trip of turtlebot')