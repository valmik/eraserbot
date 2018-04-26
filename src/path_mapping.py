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
boardWidth = 1.0 # width in meters
boardHeight = 1.3 # height in meters

# Image size
imageWidth = 0.279*0.5 # width in meters - 50% of paper
imageHeight = 0.216*0.75 # height in meters - 75% of paper

# Robot dimensions
robotWidth = 0.3 # width in meters
robotReach = 0.25 # distance from wheel base that can be scanned

# Points for 4-pt homography, from calibration
points = np.array([(331.0, 98.0), (1493.0, 62.0), (1721.0, 982.0), (181.0, 1058.0)], dtype = "float32")
pointSize = (11.0, 8.5)

# Initialize stuff
eraserbot = topic_controller.Controller() # Robot controller
bridge = CvBridge() # For processing images
rospy.wait_for_service('current_state')
state_service = rospy.ServiceProxy('current_state', StateSrv) # Get state values
rospy.wait_for_service('last_image')
image_service =  rospy.ServiceProxy('last_image', ImageSrv) # Get camera feed

# Make a folder to store images
now = datetime.datetime.now()
directory = "../images/" + now.strftime("%Y-%m-%d-%H-%M") + "/"
if not os.path.exists(directory):
    os.makedirs(directory)

# Creating state file to keep track of image coordinates
filename = directory + "states"
logfile = open(filename, 'w+')

# Values for equivalent state directions
directionUp = 0.0*math.pi # also x+
directionDown = 1.0*math.pi # also x-
directionLeft = 0.5*math.pi # also y+
directionRight = 1.5*math.pi # also y-
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
# ROBOT SHOULD BE BELOW THE BOTTOM-LEFT CORNER OF THE SCANNING AREA, FACING UPWARDS

eraserbot.closed_tank_pivot(directionUp) # correct angle
while (-1*state_service().state.y < boardWidth): # sweeping left to right until past the scanning area
    if (d > 0): # if the direction is up
        while (state_service().state.x + robotReach < boardHeight + robotReach): # while the scanning area has not reached the top of the board
            rospy.sleep(1.0)
            imageCount = take_image(imageCount) # take a picture

            print("Moving up")
            eraserbot.closed_tank_pivot(directionUp) # correct angle
            eraserbot.closed_move_straight(imageHeight) # move up one image height
            print(state_service().state)

        # this is run after reaching the top of the board
        print("Turning right")
        eraserbot.closed_tank_pivot(directionUp) # correct angle
        eraserbot.closed_move_straight(2*robotReach) # move the wheels past the board
        eraserbot.closed_tank_pivot(directionRight) # turn to face right
        eraserbot.closed_move_straight(imageWidth) # move over one image width
        eraserbot.closed_tank_pivot(directionDown) # turn to face down

        d = -1 # set direction to down
        print(state_service().state)
        print("\n")

    else: # if the direction is down
        while (state_service().state.x - robotReach > robotReach): # while above the bottom of the board
            rospy.sleep(1.0)
            imageCount = take_image(imageCount) # take a picture

            print("Moving down")
            eraserbot.closed_tank_pivot(directionDown) # correct angle
            eraserbot.closed_move_straight(imageHeight) # move down one image height
            print(state_service().state)

        # this is run after reaching the bottom of the board
        print("Turning left")
        eraserbot.closed_tank_pivot(directionDown) # correct angle
        eraserbot.closed_move_straight(2*robotReach) # move past the board one image height
        eraserbot.closed_tank_pivot(directionRight) # turn to face right
        eraserbot.closed_move_straight(imageWidth) # move over one image width
        eraserbot.closed_tank_pivot(directionUp) # turn to face up

        d = 1 # set direction to up
        print(state_service().state)
        print("\n")

logfile.close() # stop writing to the state file
print("Done scanning board!")



