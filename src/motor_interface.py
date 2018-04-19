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


if __name__ == '__main__':
    bot = Robot(1, 2)
    bot.setSpeed(150, 150)

