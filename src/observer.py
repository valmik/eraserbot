#!/usr/bin/env python

import serial
import rospy
from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import Header

import math

# http://ozzmaker.com/guide-interfacing-gyro-accelerometer-raspberry-pi-kalman-filter/
# http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
# https://ieeexplore.ieee.org/document/7014709/?reload=true
# http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/06-motion-models.pdf


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
        

class StateEstimator():
    """docstring for StateEstimator"""
    def __init__(serial_read):
        # x, y, theta
        self.pose = self.Vector3Stamped()
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = "odom"
        self.pose.vector = Vector3(0.0, 0.0, 0.0)

        # previous velocity
        self.velocity = Vector3(0.0, 0.0, 0.0)

        # Encoder values
        self.encoderLeft = 0.0
        self.encoderRight = 0.0
        self.encoderResolution = 2048.0

        # wheel radius
        self.R = 112.0 # mm
        # dist between wheels
        self.L = 205.0 # mm

        self.ser = serial_read

        self.pub = rospy.Publisher('odometry', TwistStamped, queue_size=10)
        self.rospy.init_node('observer', anonymous=True)

        
    def state_publisher(self):

        """
        This reads from serial and publishes to the odometry topic
        I'm not publishing directly to the tf2 broadcaster because 
        I've observed tf2 to be pretty slow in the past, and I want to
        read the serial as fast as possible to not fill the buffer in the
        Arduino
        """

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            read_serial = self.ser.readline()
            data = convert_from_serial(read_serial)
            self.update(data[0], data[1], data[2], data[3], data[4])
            self.publish()
            rate.sleep()


    def publish(self):

        message = TwistStamped()
        message.header = self.pose.header
        message.twist.linear = self.pose.vector
        message.twist.linear.z = 0.0
        message.twist.angular = Vector3(0.0, 0.0, self.pose.vector.z)
        self.pub.publish(message)



    def update(self, el, er, ax, ay, gz):
        """
        Updates the state using the sensor information
        Currently uses pure odometry with a first order linearization
        it does not use the imu yet
        """

        new_time = rospy.Time.now()

        # difference in radians
        dr = (er - self.encoderRight)/self.encoderResolution * 2*math.pi
        dl = (el - self.encoderLeft)/self.encoderResolution * 2*math.pi

        duration = new_time - self.pose.header.stamp
        tdiff = duration.to_nsec()/(9000000000.0)
        # convert encoder diffs to nums
        vr = dr/tdiff
        vl = dl/tdiff

        new_velocity = Vector3()
        new_position = Vector3()

        # thetadot
        new_velocity.z = (self.R/self.L)*(vr - vl)
        new_position.z = new_velocity.z*tdiff + self.pose.z

        mean_theta = (new_position.z + self.pose.z)/2.0
        new_velocity.x = (self.R/2.0)*(vr + vl) * math.cos(mean_theta)
        new_velocity.y = (self.R/2.0)*(vr + vl) * math.sin(mean_theta)

        new_position.x = new_velocity.x*tdiff + self.pose.x
        new_position.y = new_velocity.y*tdiff + self.pose.y

        self.pose.vector = new_position
        self.velocity = new_velocity


    def convert_from_serial(ser_str):
        """
        This returns the accelerometer, gyro, and encoder data

        """

        data = ser_str.split(",")
        num_data = [float(i) for i in data]
        # encl, encr, accx, accy, gyrz
        return num_data


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0',9600)
    estimator = StateEstimator(ser)
    try:
        estimator.state_publisher()
    except rospy.ROSInterruptException:
        pass



