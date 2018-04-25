#!/usr/bin/env python

# http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29

import rospy
from geometry_msgs.msg import Twist, TransformStamped
import tf2_ros

import motor_interface


def main():

    rospy.init_node('Controller')

    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)

    bot = motor_interface.Robot()


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("eraserbot", "world", rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
            rate.sleep()
            continue








if __name__ == '__main__':
    main()


