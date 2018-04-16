# Taken and lightly modified from pyimagesearch.com

# USAGE
# python transform_example.py --image images/example_01.png --coords "[(73, 239), (356, 117), (475, 265), (187, 443)]"
# python transform_example.py --image images/example_02.png --coords "[(101, 185), (393, 151), (479, 323), (187, 441)]"
# python transform_example.py --image images/example_03.png --coords "[(63, 242), (291, 110), (361, 252), (78, 386)]"
# python transform_example.py --image images/calibration_01.png --coords "[(133, 938), (603, 206), (1339, 216), (1841, 946)]"
# python transform_example.py --image images/calibration_01.png --coords "[(603, 206), (1339, 216), (1841, 946), (133, 938)]"


# import the necessary packages
from transform import four_point_transform
import numpy as np
import argparse
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image file")
ap.add_argument("-c", "--coords",
    help = "comma seperated list of source points")
args = vars(ap.parse_args())

# load the image and grab the source coordinates (i.e. the list of
# of (x, y) points)
# NOTE: using the 'eval' function is bad form, but for this example
# let's just roll with it -- in future posts I'll show you how to
# automatically determine the coordinates without pre-supplying them
image = cv2.imread(args["image"])
pts = np.array(eval(args["coords"]), dtype = "float32")

# apply the four point tranform to obtain a "birds eye view" of
# the image
warped = four_point_transform(image, pts)

# show the original and warped images
# Note: These two don't work on the Pi for some reason, but that's fine there are no errors
r = 400.0/image.shape[1]
dim = (400, int(image.shape[0]*r))
image_small = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
cv2.imshow("Original", image_small)


# https://stackoverflow.com/questions/11764575/python-2-7-3-opencv-2-4-after-rotation-window-doesnt-fit-image
def rotateAndScale(img, scaleFactor = 0.5, degreesCCW = 30):
    (oldY,oldX) = (img.shape[0], img.shape[1]) #note: numpy uses (y,x) convention but most OpenCV functions use (x,y)
    M = cv2.getRotationMatrix2D(center=(oldX/2,oldY/2), angle=degreesCCW, scale=scaleFactor) #rotate about center of image.

    #choose a new image size.
    newX,newY = oldX*scaleFactor,oldY*scaleFactor
    #include this if you want to prevent corners being cut off
    r = np.deg2rad(degreesCCW)
    newX,newY = (abs(np.sin(r)*newY) + abs(np.cos(r)*newX),abs(np.sin(r)*newX) + abs(np.cos(r)*newY))

    #the warpAffine function call, below, basically works like this:
    # 1. apply the M transformation on each pixel of the original image
    # 2. save everything that falls within the upper-left "dsize" portion of the resulting image.

    #So I will find the translation that moves the result to the center of that region.
    (tx,ty) = ((newX-oldX)/2,(newY-oldY)/2)
    M[0,2] += tx #third column of matrix holds translation, which takes effect after rotation.
    M[1,2] += ty

    rotatedImg = cv2.warpAffine(img, M, dsize=(int(newX),int(newY)))
    return rotatedImg

r = 400.0/image.shape[1]
dim = (400, int(image.shape[0]*r))
warped_small = cv2.resize(warped, dim, interpolation = cv2.INTER_AREA)
M = cv2.getRotationMatrix2D((warped_small.shape[1] / 2, warped_small.shape[0] / 2), -90, 1.0)
warped_small_rotated = rotateAndScale(warped_small, 1.0, -90.0)
cv2.namedWindow("Warped", cv2.WINDOW_NORMAL)
cv2.imshow("Warped", warped_small_rotated)
cv2.waitKey(0)



