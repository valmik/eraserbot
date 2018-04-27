# import the necessary packages
from panorama import Stitcher
import argparse
import imutils
import cv2
import numpy as np

# python stitch.py --first images/splice2_warped.png --second images/splice1_warped.png
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-f", "--first", required=True,
    help="path to the first image")
ap.add_argument("-s", "--second", required=True,
    help="path to the second image")
args = vars(ap.parse_args())

# load the two images and resize them to have a width of 400 pixels
# (for faster processing)
imageA = cv2.imread(args["first"])
imageB = cv2.imread(args["second"])
imageA = imutils.resize(imageA, width=400)
imageB = imutils.resize(imageB, width=400)

M = np.float32([[1,0,0],[0,1,50]])
imageB = cv2.warpAffine(imageB, M, (imageB.shape[1], imageB.shape[0]))

# M = np.float32([[0.966,-0.259,80],[0.259,0.966,0]])
M = np.float32([[0,-1,400],[1,0,-100]])
imageA = cv2.warpAffine(imageA, M, (imageA.shape[1], imageA.shape[0]))
 
# stitch the images together to create a panorama
stitcher = Stitcher()
(result, vis) = stitcher.stitch([imageA, imageB], showMatches=True)
 
# show the images
cv2.imshow("Image A", imageA)
cv2.imshow("Image B", imageB)
cv2.imshow("Keypoint Matches", vis)
cv2.imshow("Result", result)
cv2.waitKey(0)




