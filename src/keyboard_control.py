import math
import motor_interface
import numpy as np
import rospy
import time
import sys
import termios
import os
import tty
from eraserbot.srv import ImageSrv, ImageSrvResponse, StateSrv, StateSrvResponse
from geometry_msgs.msg import TwistStamped, Vector3

ch = ''

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

button_delay = 0.05

bot = motor_interface.Robot(1,2)

node = rospy.init_node("KeyboardControl")

while not rospy.is_shutdown():
    char = getch()
    print char

    if (char == "w"):
        bot.set_speed(150, 150)
        rospy.sleep(button_delay)

    elif (char == "a"):
        bot.set_speed(-150, 150)
        rospy.sleep(button_delay)

    elif (char == "s"):
        bot.set_speed(-150, -150)
        rospy.sleep(button_delay)

    elif (char == "d"):
        bot.set_speed(150, -150)
        rospy.sleep(button_delay)

    elif (char == "q"):
        sys.exit()

    else:
        bot.set_speed(0,0)
