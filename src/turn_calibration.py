#!/usr/bin/env python

# http://wiki.ros.org/rospy/Overview/Time

import cv2
import datetime
import math
import numpy as np
import os
import rospy
import time
import topic_controller
from cv_bridge import CvBridge, CvBridgeError
from eraserbot.srv import ImageSrv, ImageSrvResponse, StateSrv, StateSrvResponse
from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import Image
from transform import four_point_transform

# Initialize stuff
eraserbot = topic_controller.Controller() # Robot controller
bridge = CvBridge() # For processing images
rospy.wait_for_service('current_state')
state_service = rospy.ServiceProxy('current_state', StateSrv) # Get state values
rospy.wait_for_service('last_image')
image_service =  rospy.ServiceProxy('last_image', ImageSrv) # Get camera feed

# Values for equivalent state directions
directionUp = 0.0*math.pi # also x+
directionDown = 1.0*math.pi # also x-
directionLeft = 0.5*math.pi # also y+
directionRight = 1.5*math.pi # also y-

# DRIVES IN A SQUARE TO CALIBRATE ANGLES

def pause_with_warning(t):
    if (t > 3):
        rospy.sleep(t-3)
    print("Moving in less than 3 seconds!")
    rospy.sleep(3)

    
pause_with_warning(10)
eraserbot.closed_tank_pivot(directionUp)
pause_with_warning(10)
eraserbot.closed_move_straight(0.45)
pause_with_warning(10)
eraserbot.closed_tank_pivot(directionRight)
pause_with_warning(10)
eraserbot.closed_move_straight(0.45)
pause_with_warning(10)
eraserbot.closed_tank_pivot(directionDown)
pause_with_warning(10)
eraserbot.closed_move_straight(0.45)
pause_with_warning(10)
eraserbot.closed_tank_pivot(directionLeft)
pause_with_warning(10)
eraserbot.closed_move_straight(0.45)
pause_with_warning(10)
eraserbot.closed_tank_pivot(directionUp)
print("DONE WITH TEST")





