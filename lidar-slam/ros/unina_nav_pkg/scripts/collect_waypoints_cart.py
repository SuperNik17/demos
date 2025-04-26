#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion
import numpy as np
import math
import time 
import os 
import sys
import tf


if __name__ == '__main__':

    rospy.init_node('collection_node', anonymous=True)

    trans_listener = tf.TransformListener()
    
    if os.path.exists("collected_waypoints_cart.txt"):
        os.remove("collected_waypoints_cart.txt")
    f = open("collected_waypoints_cart.txt", "w")  
    r = rospy.Rate(0.2)
    while not rospy.is_shutdown(): 
        try:
            (t, q) = trans_listener.lookupTransform( 'robot/odom', 'robot/base_link', rospy.Time(0))
            f.writelines(str(t[0]) + ' ' + str(t[1]) + ' ' + str(t[2]) + ' ' +str(q[0]) + ' ' + str(q[1]) + ' ' + str(q[2]) + ' ' +str(q[3]) + '\n')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue              
        r.sleep()
    f.close()
