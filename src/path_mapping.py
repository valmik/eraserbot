#!/usr/bin/env python

# http://wiki.ros.org/rospy/Overview/Time

import rospy
import topic_controller
import math
import time
import datetime
import cv2
import os
import numpy as np
from geometry_msgs.msg import TwistStamped, Vector3
from eraserbot.srv import ImageSrv, ImageSrvResponse, StateSrv, StateSrvResponse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from transform import four_point_transform


boardSizeX = 0.75 # width of the scannable board in meters
boardSizeY = 0.5 # height of the scannable board in meters
imageSizeX = 0.23 # width of scanning area
imageSizeY = 0.23 # height of scanning area

# Points for the 4-pt homography, from calibration
pts = np.array([(331, 98), (1493, 62), (1721, 982), (181, 1058)])
size = (8.5, 11.0)

eraserbot = topic_controller.Controller()
# coordinates = [0,0,1]  # x in meters, y in meters, direction in [1,-1] for up/down

rospy.wait_for_service('current_state')
rospy.wait_for_service('last_image')
state_service = rospy.ServiceProxy('current_state', StateSrv)
image_service =  rospy.ServiceProxy('last_image', ImageSrv)

bridge = CvBridge()

now = datetime.datetime.now()
directory = "../images/" + now.strftime("%Y-%m-%d-%H-%M") + "/"
if not os.path.exists(directory):
    os.makedirs(directory)

filename = directory + "states"
logfile = open(filename, 'w+')

xi = state_service().state.x
yi = state_service().state.y
ti = state_service().state.z
d = 1 # positive for up, negative for down
count = 0; # counter for pictures taken so it can save them with different names

def take_image(c):
    print "taking image"
    state = state_service().state
    pic = image_service().image_data
    np_pic = np.array(bridge.imgmsg_to_cv2(pic, 'bgr8'))
    warped = four_point_transform(np_pic, pts, size)
    path = directory + str(c) + ".png"
    cv2.imwrite(path, warped)
    output = [c, state.x, state.y, state.z]
    output_str = ",".join([str(x) for x in output]) + "\n"
    logfile.write(output_str)
    return c+1

while (state_service().state.x < boardSizeX): # sweeping left to right until x-coord is past the scanning area
    if (d > 0): # if the direction is up
        while (state_service().state.y < boardSizeY): # while below the top of the board
            count = take_image(count)
            
            # time.sleep(0.5)

            print("Moving up")
            eraserbot.closed_tank_pivot(0.5*math.pi)
            eraserbot.closed_move_straight(imageSizeY) # move up one image height
            print(state_service().state)
            # time.sleep(2)

        # this is run after reaching the top of the board
        print("Turning right")
        eraserbot.closed_tank_pivot(0.5*math.pi)
        eraserbot.closed_move_straight(imageSizeY) # move past the board one image height
        eraserbot.closed_tank_pivot(0.0*math.pi) # turn to face right
        eraserbot.closed_move_straight(imageSizeX) # move over one image width
        eraserbot.closed_tank_pivot(1.5*math.pi) # turn to face down

        d = -1 # set direction to down
        print(state_service().state)
        ("\n")
        # time.sleep(2)

    else: # if the direction is down
        while (state_service().state.y > 0): # while above the bottom of the board
            count = take_image(count) # or equivalent capture function
            # time.sleep(0.5)

            print("Moving down")
            eraserbot.closed_tank_pivot(1.5*math.pi)
            eraserbot.closed_move_straight(imageSizeY) # move down one image height
            print(state_service().state)
            # time.sleep(2)

        # this is run after reaching the bottom of the board
        print("Turning left")
        eraserbot.closed_tank_pivot(1.5*math.pi)
        eraserbot.closed_move_straight(imageSizeY) # move past the board one image height
        eraserbot.closed_tank_pivot(0.0*math.pi) # turn to face right
        eraserbot.closed_move_straight(imageSizeX) # move over one image width
        eraserbot.closed_tank_pivot(0.5*math.pi) # turn to face up

        d = 1 # set direction to up
        print(state_service().state)
        print("\n")
        # time.sleep(2)

logfile.close()
print("Done scanning board!")



