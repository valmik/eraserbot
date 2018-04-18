#!/usr/bin/env python

import serial
import rospy
from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import Header

# http://ozzmaker.com/guide-interfacing-gyro-accelerometer-raspberry-pi-kalman-filter/
# http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

class AngleKalmanFilter():
    """docstring for KalmanFilter"""
    def __init__(self, initial_angle):
        # Process Noise Covariance (accel, gyro)
        # Since we assume the accel and gyro are independent,
        # these are just the variances
        # When multiplied by delta T, we get the actual covariance
        # Since we assume that covariance increases linearly with time
        # These are gained through measurement
        self.Q_angle = 0.01
        self.Q_gyro = 0.003

        # Observation measurement covariance
        # Since there's only one state observation (angle), the 
        # covariance is just the variance
        self.R_angle = 0.01

        # Bias, which we initially assume is zero
        self.z_bias = 0.0

        # Error covariance matrix, which we also assume starts at 0
        self.P_00 = 0.0
        self.P_01 = 0.0
        self.P_10 = 0.0
        self.P_11 = 0.0

        self.est_angle = 0.0

    def update():
        print "hi"
        


def state_publisher(ser):

    """
    This reads from serial and publishes to the odometry topic
    I'm not publishing directly to the tf2 broadcaster because 
    I've observed tf2 to be pretty slow in the past, and I want to
    read the serial as fast as possible to not fill the buffer in the
    Arduino
    """

    pub = rospy.Publisher('odometry', TwistStamped, queue_size=10)
    rospy.init_node('observer', anonymous=True)
    rate = rospy.Rate(100)


    while not rospy.is_shutdown():
        read_serial = ser.readline()
        pose = convert_from_serial(read_serial)
        pub.publish(pose)
        rate.sleep()


def convert_from_serial(ser_str):
    """
    This returns the accelerometer, gyro, and encoder data

    """

    data = ser_str.split(",")
    num_data = [float(i) for i in data]

    pose = TwistStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "odom"
    pose.twist.linear.x = num_data[0]
    pose.twist.linear.y = num_data[1]
    pose.twist.linear.z = 0
    pose.twist.angular.x = num_data[2]
    pose.twist.angular.y = num_data[3]
    pose.twist.angular.z = num_data[4]

    return pose


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0',9600)
    try:
        state_publisher(ser)
    except rospy.ROSInterruptException:
        pass



