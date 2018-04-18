#!/usr/bin/env python

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
        atexit.register(turnOffMotors)


    # For shutdown
    def turnOffMotors(self):
        self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

    # def low_level_control(currentVelocity, desiredVelocity):

    def set_speed(sl, sr):
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


