#!/usr/bin/env python

# https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library/blob/master/examples/Robot.py
# https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi/using-dc-motors


import rospy
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import atexit
import numpy as np

class Robot():
    def __init__(self, left_id, right_id):
        # create the default object without changing I2C address or frequency
        self.mh = Adafruit_MotorHAT(addr=0x60)
        self.left = self.mh.getMotor(left_id)
        self.right = self.mh.getMotor(right_id)

        # wheel radius
        self.R = 0.056 # m
        # dist between wheels
        self.L = 0.220 # m

        # Calibrated motor functions, speed = a*command + b
        # command = (speed - b)/a
        self.left_a = 0.0260
        self.left_b = 0.0007

        self.right_a = 0.0258
        self.right_b = 0.0007


        # measured minimum speed needed
        self.min_command = 20 # motor command between 0 and 255

        atexit.register(self.turnOffMotors)


    # For shutdown
    def turnOffMotors(self):
        self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

    # def low_level_control(currentVelocity, desiredVelocity):

    def set_speed(self, sl, sr):
        speed_left = max(-255, min(sl, 255))
        speed_right = max(-255, min(sr, 255))

        print "speed left: ", speed_left, "speed right: ", speed_right


        if speed_left < 0:
            self.left.setSpeed(-speed_left)
            self.left.run(Adafruit_MotorHAT.BACKWARD)
        elif speed_left == 0:
            self.left.run(Adafruit_MotorHAT.RELEASE)
        else:
            self.left.setSpeed(speed_left)
            self.left.run(Adafruit_MotorHAT.FORWARD)
        if speed_right < 0:
            self.right.setSpeed(-speed_right)
            self.right.run(Adafruit_MotorHAT.BACKWARD)
        elif speed_right == 0:
            self.right.run(Adafruit_MotorHAT.RELEASE)
        else:
            self.right.setSpeed(speed_right)
            self.right.run(Adafruit_MotorHAT.FORWARD)


    def vw_to_lr(self, vd, wd):
        """
        Converts linear and angular velocity to motor velocities
        Input is in m/s and rad/s
        Output is a motor command
        """

        # Find desired speed in rad/s
        vl_rad = ((2.0/self.R)*vd - (self.L/self.R)*wd)/2.0
        vr_rad = (self.L/self.R)*wd + vl_rad

        # Convert rad/s to motor command
        # command = (speed - b)/a
        vl = (vl_rad - self.left_b)/(self.left_a)
        vr = (vr_rad - self.right_b)/(self.right_a)
        # vl = vl/self.max_speed*255
        # vr = vr/self.max_speed*255

        # Scale to +- 255
        if abs(vl) > 255:
            vr = vr/vl*255
            vl = vl/vl*255
        if abs(vr) > 255:
            vl = vl/vr*255
            vr = vr/vr*255

        return vl, vr

    def lr_to_vw(self, vl, vr):
        """
        Convertes motor velocities to linear and angular velocities
        Input is a motor command, output is robot velocities in m/s and rad/s
        """

        # Convert command to rad/s
        # speed = a*command + b
        vl = float(np.sign(vl))*(self.left_a * abs(vl) + self.left_b)
        vr = float(np.sign(vr))*(self.right_a * abs(vr) + self.right_b)
        # vl = vl/255*self.max_speed
        # vr = vr/255*self.max_speed

        w = (self.R/self.L)*(vr - vl)
        v = (self.R/2.0)*(vl + vr)

        return v, w

    def scale_velocity(self, v, w):
        """
        scales velocity so that it's achievable
        """

        l, r = self.vw_to_lr(v, w)
        vn, wn = self.lr_to_vw(l, r)

        return vn, wn

    def test_speed(self):
        """
        user input for testing motors
        """

        l = int(raw_input("left motor value: "))
        r = int(raw_input("right motor value: "))

        self.set_speed(l, r)
        rospy.sleep(3)
        print "stopping"
        self.turnOffMotors()
        
    def test_radps(self):
        """
        command with angular velocities instead
        """

        l = float(raw_input("left motor speed: "))
        r = float(raw_input("right motor speed: "))

        l = (l - self.left_b)/(self.left_a)
        r = (r - self.right_b)/(self.right_a)

        self.set_speed(int(l), int(r))
        rospy.sleep(3)
        print "stopping"
        self.turnOffMotors()


if __name__ == '__main__':
    bot = Robot(1, 2)
    while True:
        bot.test_radps()

