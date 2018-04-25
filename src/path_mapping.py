#!/usr/bin/env python

# http://wiki.ros.org/rospy/Overview/Time

import rospy
import topic_controller
import math
import time
from eraserbot.srv import ImageSrv, ImageSrvResponse, StateSrv, StateSrvResponse
from cv_bridge import CvBridge, CvBridgeError


boardSizeX = 0.75 # width of the scannable board in meters
boardSizeY = 0.5 # height of the scannable board in meters
imageSizeX = 0.23 # width of scanning area
imageSizeY = 0.23 # height of scanning area

eraserbot = topic_controller.Controller()
coordinates = [0,0,1]  # x in meters, y in meters, direction in [1,-1] for up/down


rospy.wait_for_service('current_state')
state_service = rospy.ServiceProxy('current_state', StateSrv)

current_state = state_service().state


while (coordinates[0] < boardSizeX): # sweeping left to right until x-coord is past the scanning area
    if (coordinates[2] > 0): # if the direction is down
        while (coordinates[1] < boardSizeY): # while below the top of the board
            print("Snap!") # or equivalent capture function
            time.sleep(0.5)

            print("Moving up")
            eraserbot.move_straight(imageSizeY) # move up one image height
            coordinates[1] = coordinates[1] + imageSizeY # increase coordinates by one image height
            print(coordinates)
            time.sleep(0.5)

        # this is run after reaching the top of the board
        print("Turning right")
        eraserbot.move_straight(imageSizeY) # move past the board one image height
        eraserbot.tank_pivot(-0.5*math.pi) # turn right pi/2
        eraserbot.move_straight(imageSizeX) # move over one image width
        eraserbot.tank_pivot(-0.5*math.pi) # turn right pi/2

        # update coordinates
        coordinates[0] = coordinates[0] + imageSizeX
        coordinates[1] = coordinates[1] + imageSizeY
        coordinates[2] = -1 # set direction to down
        print(coordinates)
        print("\n")

    else: # if the direction is up
        while (coordinates[1] > 0): # while above the bottom of the board
            print("Snap!") # or equivalent capture function
            time.sleep(0.5)

            print("Moving down")
            eraserbot.move_straight(imageSizeY) # move down one image height
            coordinates[1] = coordinates[1] - imageSizeY # increase coordinates by one image height
            print(coordinates)
            time.sleep(0.5)

        # this is run after reaching the bottom of the board
        print("Turning left")
        eraserbot.move_straight(imageSizeY) # move past the board one image height
        eraserbot.tank_pivot(0.5*math.pi) # turn left pi/2
        eraserbot.move_straight(imageSizeX) # move over one image width
        eraserbot.tank_pivot(0.5*math.pi) # turn left pi/2

        # update coordinates
        coordinates[0] = coordinates[0] + imageSizeX
        coordinates[1] = coordinates[1] - imageSizeY
        coordinates[2] = 1 # set direction to up
        print(coordinates)
        print("\n")

print("Done scanning board!")






