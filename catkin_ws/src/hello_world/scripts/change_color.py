#!/usr/bin/env python3


import rospy
import random
from std_srvs.srv import Empty

def change_color():
    rospy.init_node('change_color', anonymous=True)
    #Setting random values from 0-255 in the color parameters 
    rospy.set_param('/background_r',random.randint(0,255))
    rospy.set_param('/background_b',random.randint(0,255)) 
    rospy.set_param('/background_g',random.randint(0,255)) 
    

    # waiting for service /reset
    rospy.wait_for_service('/reset')

    # calling /reset service
    try:
        serv = rospy.ServiceProxy('/reset', Empty)
        resp = serv()
        rospy.loginfo('service executed')
    except rospy.ServiceException as e:
        rospy.loginfo('Service call failed', e)
        
    rospy.spin()

if __name__ == '__main__':
    change_color()