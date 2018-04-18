#!/usr/bin/env python

import serial
import rospy
from geometry_msgs.msg import TwistStamped

"""
This reads from serial and publishes to the odometry topic
I'm not publishing directly to the tf2 broadcaster because 
I've observed tf2 to be pretty slow in the past, and I want to
read the serial as fast as possible to not fill the buffer in the
Arduino
"""


def state_publisher(ser):
    pub = rospy.Publisher('odometry', TwistStamped, queue_size=10)
    rospy.init_node('observer', anonymous=True)
    rospy.Rate(100)
    while not rospy.is_shutdown():
        read_serial = ser.readline()
        pose = convert_from_serial()
        pub.publish(pose)
        rate.sleep()


def convert_from_serial(ser_str):
    pose = TwistStamped()
    pose.header.stamp = rospy.get_time()
    pose.header.frame_id = "odom"
    pose.twist.linear.x = 0
    pose.twist.linear.y = 0
    pose.twist.linear.z = 0
    position.twist.angular.x = 0
    position.twist.angular.y = 0
    position.twist.angular.z = 0


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM4',9600)
    try:
        state_publisher(ser)
    except rospy.ROSInterruptException:
        pass



