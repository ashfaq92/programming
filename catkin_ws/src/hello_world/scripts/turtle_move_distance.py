#!/usr/bin/env python3

import rospy
import random
import sys
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose



robot_x = 0

-p-p

def pose_callback(pose):
    global robot_x

    rospy.loginfo("Robot X = %f : Y = %f : Z = %f", pose.x, pose.y, pose.theta)

    robot_x = pose.x

def move_turtle(lin_vel, ang_vel, distance):
    global robot_x

    rospy.init_node("turtle_mover_get_pose", anonymous=False)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    rate = rospy.Rate(10)

    vel = Twist()

    while not rospy.is_shutdown():
        vel.linear.x = lin_vel
        vel.linear.y = 0
        vel.linear.y = 0

        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel

        # rospy.loginfo('linear_vel', lin_vel, 'angular_vel', ang_vel)
        rospy.loginfo('linear vel = %f, angular vel = %f', vel.linear.x, vel.angular.z)

        # distance check
        if robot_x >= distance:
            rospy.loginfo("destination reached")
            rospy.logwarn("stopping robot")
            break

        pub.publish(vel)

        rate.sleep()

if __name__ == '__main__':
    change_color()
    move_turtle(
        float(sys.argv[1]), 
        float(sys.argv[2]),
        float(sys.argv[3]),
    )

