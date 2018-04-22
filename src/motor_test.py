#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, Vector3, Vector3Stamped
from std_msgs.msg import Header
import math
import sys
import datetime
import motor_interface

def motorTest(motor, logfile):

    bot = motor_interface.Robot(1,2)
    rate = rospy.Rate(0.1)
    
    def motorTestLeft(speed):

        output = [speed]
        
        def motorCallbackLeft(msg):
            output.append(msg.vector.x)

        bot.set_speed(speed, 0)
        sub = rospy.Subscriber('motor_test', Vector3Stamped, motorCallbackLeft)
        start_time = rospy.get_time()
        while True:
            if len(output) > 10:
                break
        sub.unregister()
        bot.turnOffMotors()
        print output
        output_str = ",".join([str(x) for x in output]) + "\n"
        logfile.write(output_str)

    def motorTestRight(speed):

        output = [speed]
        
        def motorCallbackRight(msg):
            output.append(msg.vector.y)

        bot.set_speed(0, speed)
        sub = rospy.Subscriber('motor_test', Vector3Stamped, motorCallbackRight)
        start_time = rospy.get_time()
        while True:
            if len(output) > 10:
                break
        sub.unregister()
        bot.turnOffMotors()
        print output
        output_str = ",".join([str(x) for x in output]) + "\n"
        logfile.write(str(output))

    # go from 0 to 255 in multiples of 15
    speed_range = range(0, 256, 15)

    if motor == "left":
        for s in speed_range:
            motorTestLeft(s)
        
    if motor == "right":
        for s in speed_range:
            motorTestRight(s)
            
    return

if __name__ == '__main__':

    script, motor = sys.argv

    if motor != "left" and motor != "right":
        print "Argument should be 'left' or 'right'"
        sys.exit()
    now = datetime.datetime.now()
    filename = "../logs/" + now.strftime("%Y-%m-%d-%H-%M") + " " + motor
    logfile = open(filename, 'w+')

    rospy.init_node('motor_testing', anonymous=True)
    rospy.on_shutdown(logfile.close)
    motorTest(motor, logfile)




