#!/usr/bin/env python

# https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library/blob/master/examples/Robot.py
# https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi/using-dc-motors


import rospy
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import atexit

class Robot():
    def __init__(self, left_id, right_id):
        # create the default object without changing I2C address or frequency
        self.mh = Adafruit_MotorHAT(addr=0x60)
        self.left = self.mh.getMotor(left_id)
        self.right = self.mh.getMotor(right_id)

        # wheel radius
        self.R = 0.112 # m
        # dist between wheels
        self.L = 0.205 # m


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
        """
        vl = ((2.0/self.R)*vd - (self.L/self.R)*wd)/2.0
        vr = (self.L/self.R)*wd + vl

        if vl > 255:
            vr = vr/vl*255
            vl = 255
        if vr > 255:
            vl = vl/vr*255
            vr = 255

        return vl, vr

    def lr_to_vw(self, vl, vr):
        """
        Convertes motor velocities to linear and angular velocities
        """

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

        l = int(raw_input("left motor value"))
        r = int(raw_input("right motor value"))

        self.set_speed(l, r)
        rospy.sleep(3)
        print "stopping"
        self.turnOffMotors()
        



if __name__ == '__main__':
    bot = Robot(1, 2)
    while True:
        bot.test_speed()

