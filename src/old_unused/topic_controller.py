#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, Vector3

import math
import time
import numpy as np
import motor_interface

class Controller():
    """docstring for Controller"""
    def __init__(self):
        rospy.init_node("Controller")

        self.bot = motor_interface.Robot(1, 2)

        self.odom_sub = rospy.Subscriber('/odometry', TwistStamped, self.update_state, queue_size = 1)

        self.state = Vector3(0.0,0.0,0.0)

        self.k1 = 0.5
        self.k2 = 1.0
        self.k3 = 1.0

        self.previous = Vector3(0.0, 0.0, 0.0)


    def update_state(self, msg):
        self.state.x = msg.twist.linear.x
        self.state.y = msg.twist.linear.y
        self.state.z = msg.twist.angular.z

        
    def move_to_pose(self, desired_pose):
        """
        desired_pose: a Vector3 containing desired x,y,theta
        THIS FUNCTION DOES NOT WORK CORRECTLY
        """

        while not rospy.is_shutdown():
            current_pose = self.state
            x = current_pose.x
            y = current_pose.y
            t = current_pose.z

            xr = desired_pose.x
            yr = desired_pose.y
            tr = desired_pose.z

            ep = (math.sqrt((xr - x)**2 + (yr - y)**2))
            et = (tr - t)

            if ep < 0.01 and et < 0.1:
                return

            vri = ep/0.1
            wri = et/0.1

            vr, wr = self.bot.scale_velocity(vri, wri)

            vd, wd = self.control(desired_pose, current_pose, vr, wr)

            l, r = self.bot.vw_to_lr(vd, wd)
            self.bot.set_speed(int(l), int(r))


    def control(self, desired_pose, current_pose, vr, wr):
        """
        Contains the actual controller
        """

        x = current_pose.x
        y = current_pose.y
        t = current_pose.z

        xr = desired_pose.x
        yr = desired_pose.y
        tr = desired_pose.z

        ex = (math.cos(t) - math.sin(t))*(xr - x)
        ey = (math.sin(t) + math.cos(t))*(yr - y)

        u1 = -self.k1*ex
        u2 = self.k2*vr*np.sinc(tr - t)*ey - self.k3*(tr - t)

        vd = math.cos(tr - t)*vr - u1
        wd = wr - u2

        print ex, ey, vd, wd

        return vd, wd


    def move_straight(self, dist):
        """
        dist: Desired distance in meters
        Moves [dist] meters forwards/backwards in a straight line
        Open loop function
        """

        mvel = 200 # speed that wheel will move
        vel = self.bot.lr_to_vw(mvel, mvel) # find the equivalent speed in m/s
        t = dist/vel[0] # travel time
	if (dist < 0): # correct for negative values
            t = -1*t
            mvel = -1*mvel

        self.bot.set_speed(mvel,mvel) # drive
        time.sleep(t) # keep driving until distance should be reached
        self.bot.turnOffMotors() # stop driving


    def tank_pivot(self, theta):
        """
        theta: Desired angle in radians
        Pivots [theta] radians counterclockwise/clockwise around the center of the wheels
        Open loop function
        """

        dw = 0.2175 # width of wheel base in meters
        mvel = 100 # speed that wheel will move
        
        dist = dw/2*theta # arc length that wheels should move
        vel = self.bot.lr_to_vw(mvel, mvel) # find the equivalent speed in m/s
        t = dist/vel[0] # travel time
	if (theta < 0): # correct for negative values
            t = -1*t
            mvel = -1*mvel

        self.bot.set_speed(-1*mvel, mvel) # pivot about center of wheels
        time.sleep(t) # keep turning until angle should be reached
        self.bot.turnOffMotors() # stop turning


def main():

    control = Controller()
    desired_pose = Vector3(1.0, 0, 0)
    control.move_to_pose(desired_pose)


if __name__ == '__main__':
    main()


