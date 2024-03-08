#!/usr/bin/env python3

import rospy
import sys
import random

from geometry_msgs.msg import Twist

def move_turtle():
    rospy.init_node('turtle_mover', anonymous=False)
    
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)

    vel = Twist()

   
    
    while not rospy.is_shutdown():

        vel.linear.x = random.random()
        vel.linear.y = random.random()
        vel.linear.z = random.random()

        vel.angular.x = random.random()
        vel.angular.y = random.random()
        vel.angular.z = random.random()

        rospy.loginfo('linear vel = %f, angular vel = %f', vel.linear.x, vel.angular.z)

        pub.publish(vel)
        
        rate.sleep()


if __name__ == '__main__':
    move_turtle()
