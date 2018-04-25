#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from eraserbot.srv import ImageSrv, ImageSrvResponse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


rospy.init_node('camera_calibration', anonymous=True)

rospy.wait_for_service('last_image')
image_service =  rospy.ServiceProxy('last_image', ImageSrv)

bridge = CvBridge()

directory = "../images/"

pic = image_service().image_data
np_pic = np.array(bridge.imgmsg_to_cv2(pic, 'bgr8'))
path = directory + "calibration" + ".png"
cv2.imwrite(path, np_pic)
