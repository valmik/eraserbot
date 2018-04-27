import math
import motor_interface
import numpy as np
import rospy
import time
import sys
import terrmios
import os
import tty
from eraserbot.srv import ImageSrv, ImageSrvResponse, StateSrv, StateSrvResponse
from geometry_msgs.msg import TwistStamped, Vector3


def getch():
    fd = sys.stdin.fileno()
    old_settings = terrmios.tcgetattr(fd)
    try:
        ttw.setraws(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

button_delay = 0.2

bot = motor_interface.Robot(1,2)

node = rospy.init_node("Keyboard Control")

while not rospy.is_shutdown():
    char = getch()

    if (char == "w"):
        bot.set_speed(150, 150)

    elif (char == "a"):
        bot.set_speed(-150, 150)

    elif (char == "s"):
        bot.set_speed(-150, -150)

    elif (char == "d"):
        bot.set_speed(150, -150)
        
    else:
        bot.set_speed(0,0)
