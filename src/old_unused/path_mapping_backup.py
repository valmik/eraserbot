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

# Board size
boardSizeX = 0.75 # width
boardSizeY = 0.5 # height

# Image size
imageSizeX = 0.23 # width
imageSizeY = 0.23 # height

# Points for 4-pt homography, from calibration
points = np.array([(331.0, 98.0), (1493.0, 62.0), (1721.0, 982.0), (181.0, 1058.0)], dtype = "float32")
pointSize = (11.0, 8.5)

# Initialize stuff
eraserbot = topic_controller.Controller()
bridge = CvBridge()
rospy.wait_for_service('current_state')
rospy.wait_for_service('last_image')
state_service = rospy.ServiceProxy('current_state', StateSrv)
image_service =  rospy.ServiceProxy('last_image', ImageSrv)

# Make a folder to store images
now = datetime.datetime.now()
directory = "../images/" + now.strftime("%Y-%m-%d-%H-%M") + "/"
if not os.path.exists(directory):
    os.makedirs(directory)

# Creating state file to keep track of image coordinates
filename = directory + "states"
logfile = open(filename, 'w+')

d = 1 # positive for up, negative for down
imageCount = 0 # counter for pictures taken so it can save them with different names


def take_image(count):
    # takes a picture, writes to a state file, and increments the image counter
    print "Snap!"
    pic = image_service().image_data # get image from webcam
    cv_pic = bridge.imgmsg_to_cv2(pic, 'bgr8')
    path = directory + str(count) + ".png"
    warped = four_point_transform(cv_pic, points, pointSize)
    cv2.imwrite(path, warped) # save the transformed image to the folder
    state = state_service().state
    output = [count, state.x, state.y, state.z]
    output_str = ",".join([str(x) for x in output]) + "\n"
    logfile.write(output_str) # write the image number and state to a state file
    return count+1


# START OF SCRIPT

eraserbot.closed_tank_pivot(0.5*math.pi) # correct angle
while (state_service().state.x < boardSizeX): # sweeping left to right until x-coord is past the scanning area
    if (d > 0): # if the direction is up
        while (state_service().state.y < boardSizeY): # while below the top of the board
            imageCount = take_image(imageCount) # take a picture
            # time.sleep(0.5)

            print("Moving up")
            eraserbot.closed_tank_pivot(0.5*math.pi) # correct angle
            eraserbot.closed_move_straight(imageSizeY) # move up one image height
            print(state_service().state)
            # time.sleep(2)

        # this is run after reaching the top of the board
        print("Turning right")
        eraserbot.closed_tank_pivot(0.5*math.pi) # correct angle
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
            imageCount = take_image(imageCount) # take a picture
            # time.sleep(0.5)

            print("Moving down")
            eraserbot.closed_tank_pivot(1.5*math.pi) # correct angle
            eraserbot.closed_move_straight(imageSizeY) # move down one image height
            print(state_service().state)
            # time.sleep(2)

        # this is run after reaching the bottom of the board
        print("Turning left")
        eraserbot.closed_tank_pivot(1.5*math.pi) # correct angle
        eraserbot.closed_move_straight(imageSizeY) # move past the board one image height
        eraserbot.closed_tank_pivot(0.0*math.pi) # turn to face right
        eraserbot.closed_move_straight(imageSizeX) # move over one image width
        eraserbot.closed_tank_pivot(0.5*math.pi) # turn to face up

        d = 1 # set direction to up
        print(state_service().state)
        print("\n")
        # time.sleep(2)

logfile.close() # stop writing to the state file
print("Done scanning board!")



