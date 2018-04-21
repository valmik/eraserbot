#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, Vector3, Vector3Stamped
from std_msgs.msg import Header
import math
import sys
import datetime
import motor_interface

def motorTest(motor, logfile):

    rospy.init_node('motor_testing', anonymous=True)

    bot = motor_interface.Robot(1,2)

    def motorTestLeft(speed):

        output = [speed]

        def motorCallbackLeft(msg):
            output.append(msg.vector.x)

        bot.set_speed(speed, 0)
        sub = rospy.Subscriber('motor_test', Vector3Stamped, motorTestLeft, queue_size = 1)
        rospy.sleep(3)
        sub.unregister()

        logfile.write(str(output))

    def motorTestRight(speed):

        output = [speed]

        def motorCallbackRight(msg):
            output.append(msg.vector.y)

        bot.set_speed(0, speed)
        sub = rospy.Subscriber('motor_test', Vector3Stamped, motorTestRight, queue_size = 1)
        rospy.sleep(3)
        sub.unregister()

        logfile.write(str(output))



    # go from 0 to 255 in multiples of 15
    speed_range = range(0, 256, 15)

    if motor == "left":
        for s in speed_range:
            motorTestLeft(s)
        
    if motor == "right":
        for s in speed_range:
            motorTestRight(s)

    shutdown(logfile)
    return


def shutdown(logfile):
    logfile.close()
    rospy.sleep(1)


if __name__ == '__main__':

    script, motor = sys.argv

    if motor != "left" and motor != "right":
        print "Argument should be 'left' or 'right'"
        return
    now = datetime.datetime.now()
    filename = "../logs/" + now.strftime("%Y-%m-%d-%H-%M") + " " + motor
    logfile = open(filename, 'a')

    motor_test(motor, logfile)




