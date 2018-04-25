# Taken and modified from pyimagesearch.com

import numpy as np
import cv2

def four_point_transform(image, pts, size):
    """
    Do a 4-point homography transformation (with scaling)

    image: opencv image
    pts: tuple of (x,y) points (in pixels) of the top left, top right, 
        bottom right, bottom left corners IN THAT ORDER
    size: tuple of (width, length)

    """

    # obtain a consistent order of the points and unpack them
    # individually
    rect = pts
    (tl, tr, br, bl) = rect # 100 pixels per inch

    print tl, tr, br, bl

    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
    print "maxWidth", maxWidth

    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    print "maxHeight", maxHeight


    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")

    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))

    # Scale to the real-world size
    dim = (int(size[0] * 200), int(size[1]*200))
    warped = cv2.resize(warped, dim, interpolation = cv2.INTER_AREA)

    # return the warped image
    return warped


