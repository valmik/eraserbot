#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import atexit

# create the default object without changing I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# For shutdown
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

def low_level_control(currentVelocity, desiredVelocity):

