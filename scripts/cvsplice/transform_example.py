# Taken and lightly modified from pyimagesearch.com

# Useful links
# https://www.pyimagesearch.com/2014/01/20/basic-image-manipulations-in-python-and-opencv-resizing-scaling-rotating-and-cropping/


# USAGE
# python transform_example.py --image images/example_01.png --coords "[(73, 239), (356, 117), (475, 265), (187, 443)]"
# python transform_example.py --image images/example_02.png --coords "[(101, 185), (393, 151), (479, 323), (187, 441)]"
# python transform_example.py --image images/example_03.png --coords "[(63, 242), (291, 110), (361, 252), (78, 386)]"
# python transform_example.py --image images/calibration_01.png --coords "[(133, 938), (603, 206), (1339, 216), (1841, 946)]"
# python transform_example.py --image images/splice_calibration.png --coords "[(207, 120), (599, 126), (709, 396), (141, 394)]" --name "calibration_warped.png"
# python transform_example.py --image images/splice1.png --coords "[(207, 120), (599, 126), (709, 396), (141, 394)]" --name "splice1_warped.png"
# python transform_example.py --image images/splice2.png --coords "[(207, 120), (599, 126), (709, 396), (141, 394)]" --name "splice2_warped.png"
# python transform_example.py --image images/calibration.png --coords "[(331, 98), (1493, 62), (1721, 982), (181, 1058)]"


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
ap.add_argument( "-n", "--name", help = "name to save the new file")
args = vars(ap.parse_args())

print args

# load the image and grab the source coordinates (i.e. the list of
# of (x, y) points)
# NOTE: using the 'eval' function is bad form, but for this example
# let's just roll with it -- in future posts I'll show you how to
# automatically determine the coordinates without pre-supplying them
image = cv2.imread(args["image"])
pts = np.array(eval(args["coords"]), dtype = "float32")
# real = np.array([(0, 0), (11.0, 0), (11.0, 8.5), (0, 8.5)])
size = (8.5, 11.0)

# apply the four point tranform to obtain a "birds eye view" of
# the image
warped = four_point_transform(image, pts, size)

# resize
dim = (warped.shape[0]/3, warped.shape[1]/3)
warped = cv2.resize(warped, dim, interpolation = cv2.INTER_AREA)

print args.get("name")
if not (args.get("name") == None):
    path = args["image"]
    path_list = path.split('/')
    path_list[-1] = args.get("name")
    new_path = '/'.join(path_list)
    cv2.imwrite(new_path, warped)

# show the original and warped images
# Note: These two don't work on the Pi for some reason, but that's fine there are no errors
cv2.imshow("Original", image)
cv2.imshow("Warped", warped)
cv2.waitKey(0)
