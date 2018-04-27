#!/usr/bin/env python

import math
import motor_interface
import numpy as np
import rospy
import time
from eraserbot.srv import ImageSrv, ImageSrvResponse, StateSrv, StateSrvResponse
from geometry_msgs.msg import TwistStamped, Vector3


class Controller():
    def __init__(self):
        #rospy.init_node("Controller")
        self.bot = motor_interface.Robot(1, 2)
        rospy.wait_for_service('current_state')
        self.state_service = rospy.ServiceProxy('current_state', StateSrv)


    def closed_move_straight(self, dist, maxSpeed):
        # dist: Desired distance in meters
        # Moves [dist] meters forwards/backwards in a straight line
        # Closed loop function

        xi = self.state_service().state.x # get the original x position
        yi = self.state_service().state.y # get the original y position

        # initialize these things
        x = xi # x position to be updated
        y = yi # y position to be updated
        travel = 0 # distance traveled from original position
        mvel = maxSpeed # motor speed
        
        while (abs(travel - abs(dist)) > 0.01): # while distance is >1 cm from the desired distance
            if (abs(travel - abs(dist)) < 0.05): # slow down when <5 cm away
                mvel = maxSpeed / 2
            
            if (travel < abs(dist)):
                self.bot.set_speed(mvel, mvel) # assumes will drive in straight line
            else:
                self.bot.set_speed(-1*mvel,-1*mvel) # drive backwards for negative distances

            # get updated position and travel distance
            x = self.state_service().state.x
            y = self.state_service().state.y
            travel = math.sqrt((x - xi)**2 + (y - yi)**2)
            print(travel)

        self.bot.turnOffMotors() # when reached distance, stop moving
        print("done moving")


    def closed_tank_pivot(self, theta, maxSpeed):
        # theta: Desired angle position in radians
        # Rotates until 
        # Closed loop function
        # THIS TAKES DIFFERENT INPUTS THAN open_tank_pivot
        
        # initialize these things
        t = self.state_service().state.z # x position to be updated
        dist = (t - theta) % (2*math.pi) # angular offset, positive is too far counterclockwise
        
        mvel = maxSpeed # motor speed

        while (abs(dist) > 0.04): # while >0.04 rad from desired angular position
            if (abs(dist) < 0.5): # slow down when <0.5 rad away
                mvel = maxSpeed / 2

            if (dist > 0): # if too far counterclockwise
                self.bot.set_speed(mvel, -1*mvel) # turn clockwise
            else: # if too far clockwise
                self.bot.set_speed(-1*mvel, mvel) # turn counterclockwise

            t = self.state_service().state.z # get updated position
            dist = ((t - theta + math.pi) % (2*math.pi)) - math.pi # recalculate angular offset
            print(self.state_service().state.z)
        
        self.bot.turnOffMotors() # when reached distance, stop moving
