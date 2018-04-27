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


#    def open_move_straight(self, dist):
        # dist: Desired distance in meters
        # Moves [dist] meters forwards/backwards in a straight line
        # Open loop function

#        mvel = 200 # speed that wheel will move
  #      vel = self.bot.lr_to_vw(mvel, mvel) # find the equivalent speed in m/s
  #      t = dist/vel[0] # travel time
   #     if (dist < 0): # correct for negative values
   #         t = -1*t
   #         mvel = -1*mvel

#        self.bot.set_speed(mvel,mvel) # drive
  #      time.sleep(t) # keep driving until distance should be reached
    #    self.bot.turnOffMotors() # stop driving


    def closed_move_straight(self, dist, max_speed=220):
        # dist: Desired distance in meters
        # Moves [dist] meters forwards/backwards in a straight line
        # Closed loop function

        xi = self.state_service().state.x # get the original x position
        yi = self.state_service().state.y # get the original y position

        # initialize these things
        x = xi # x position to be updated
        y = yi # y position to be updated
        travel = 0 # distance traveled from original position
        mvel = max_speed # motor speed
        
        while (abs(travel - dist) > 0.01): # while distance is >1 cm from the desired distance
            if (abs(travel - dist) < 0.05): # slow down when <5 cm away
                mvel = int(max_speed/2)
            
            if (travel < dist):
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


#    def open_tank_pivot(self, theta):
 #       # theta: Desired anglular change in radians
  #      # Pivots [theta] radians counterclockwise/clockwise around the center of the wheels
        # Open loop function

    #    dw = 0.2175 # width of wheel base in meters
      #  mvel = 100 # speed that wheel will move
        
 #       dist = dw/2*theta # arc length that wheels should move
   #     vel = self.bot.lr_to_vw(mvel, mvel) # find the equivalent speed in m/s
     #   t = dist/vel[0] # travel time
    #    if (theta < 0): # correct for negative values
    #        t = -1*t
      #      mvel = -1*mvel

#        self.bot.set_speed(-1*mvel, mvel) # pivot about center of wheels
  #      time.sleep(t) # keep turning until angle should be reached
    #    self.bot.turnOffMotors() # stop turning


    def closed_tank_pivot(self, theta, max_speed=200):
        # theta: Desired angle position in radians
        # Rotates until 
        # Closed loop function
        # THIS TAKES DIFFERENT INPUTS THAN open_tank_pivot
        
        # initialize these things
        t = self.state_service().state.z # x position to be updated
        dist = (t - theta) % (2*math.pi) # angular offset, positive is too far counterclockwise
        
        mvel = max_speed # motor speed

        while (abs(dist) > 0.04): # while >0.01 rad from desired angular position
            if (abs(dist) < 0.5): # slow down when <0.5 rad away
                mvel = int(max_speed/3.0)

            if (dist > 0): # if too far counterclockwise
                self.bot.set_speed(mvel, -1*mvel) # turn clockwise
            else: # if too far clockwise
                self.bot.set_speed(-1*mvel, mvel) # turn counterclockwise

            t = self.state_service().state.z # get updated position
            dist = ((t - theta + math.pi) % (2*math.pi)) - math.pi # recalculate angular offset
            print(self.state_service().state.z)
        
        self.bot.turnOffMotors() # when reached distance, stop moving


#    def go_home(self):
 #       # DOESN'T WORK
 #       
   #     x = self.state_service().state.x
   #     y = self.state_service().state.y
     #   dist = math.sqrt(x**2 + y**2)
        
    #    while (abs(dist) > 0.01):
      #      self.closed_tank_pivot(0.5*math.pi - np.arctan(x / y))
        #    self.closed_move_straight(dist)
            
          #  x = self.state_service().state.x
   #         y = self.state_service().state.y
     #       dist = math.sqrt(x**2 + y**2)
       #     print(self.state_service().state)
         #   time.sleep(5)
        
        #self.closed_tank_pivot(0)




