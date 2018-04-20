#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, Vector3

import math
import numpy as np
import motor_interface

class Controller():
    """docstring for Controller"""
    def __init__(self):
        rospy.init_node("Controller")

        self.bot = motor_interface.Robot()

        self.odom_sub = rospy.Subscriber('/odometry', TwistStamped, self.update_state)

        self.state = Vector3(0.0,0.0,0.0)

        self.k1 = 1.0
        self.k2 = 1.0
        self.k3 = 1.0

    def update_state(self, msg):
        self.state.x = msg.twist.linear.x
        self.state.y = msg.twist.linear.y
        self.state.z = msg.twist.angular.z

        
    def move_to_pose(self, desired_pose):
        """
        Pose is a Vector3 containing desired x,y,theta
        """

        while True:
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
            self.bot.set_speed(l, r)



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
        u2 = k2*vr*np.sinc(tr - t)*ey - k3*(tr - t)

        vd = math.cos(tr - t)*vr - u1
        wd = wr - u2

        return vd, wd


def main():

    control = Controller()
    desired_pose = Vector3(1.0, 0, 0)
    control.move_to_pose(desired_pose)


if __name__ == '__main__':
    main()


