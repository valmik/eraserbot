 #!/usr/bin/env python

import numpy as np
import cv2
import imutils
from boardsplice import Stitcher

pixels_per_cm = 10
border = 500

directory = "images/small_data/"
statefile = open((directory + "states"), 'r')

picture_dict = {}
picture_list = []

for line in statefile:
    line_list = line.split(',')
    line_list[-1] = line_list[-1][0:-1]
    if len(line_list) < 4: break
    xyz = [float(x) for x in line_list[1:]]
    picture_list.append(line_list[0])
    picture_dict[line_list[0]] = tuple(xyz)

# print picture_list
# print picture_dict

minx = 0
maxx = 0
miny = 0
maxy = 0

# Image bounds in robot frame
ix1 = 0.515
ix2 = 0.300
iy1 = 0.140
iy2 = -0.140

for pic in picture_list:
    # position and orientation of the robot with respect to the world frame
    x_rw, y_rw, z_rw = picture_dict[pic]

    # Top left
    x_ir1 = np.cos(z_rw)*ix1 - np.sin(z_rw)*iy1 + x_rw
    y_ir1 = np.sin(z_rw)*ix1 + np.cos(z_rw)*iy1 + y_rw

    # Top right
    x_ir2 = np.cos(z_rw)*ix1 - np.sin(z_rw)*iy2 + x_rw
    y_ir2 = np.sin(z_rw)*ix1 + np.cos(z_rw)*iy2 + y_rw

    # Bottom left
    x_ir3 = np.cos(z_rw)*ix2 - np.sin(z_rw)*iy2 + x_rw
    y_ir3 = np.sin(z_rw)*ix2 + np.cos(z_rw)*iy2 + y_rw

    # Bottom right
    x_ir4 = np.cos(z_rw)*ix2 - np.sin(z_rw)*iy1 + x_rw
    y_ir4 = np.sin(z_rw)*ix2 + np.cos(z_rw)*iy1 + y_rw

    minx = min(minx, x_ir1, x_ir2, x_ir3, x_ir4)
    maxx = max(maxx, x_ir1, x_ir2, x_ir3, x_ir4)

    miny = min(miny, y_ir1, y_ir2, y_ir3, y_ir4)
    maxy = max(maxy, y_ir1, y_ir2, y_ir3, y_ir4)
    
# print minx, maxx, miny, maxy

# A centimeter should be 10 pixels
width = int(np.ceil((maxx - minx)*pixels_per_cm*100))
height = int(np.ceil((maxy - miny)*pixels_per_cm*100))

full_img = np.zeros((width,height,3), np.uint8)
full_img = cv2.copyMakeBorder(full_img, border, border, border, border, cv2.BORDER_CONSTANT, value = [0,0,0])

image = cv2.imread(directory + picture_list[0] + ".png")
# image[0:50,0:50,0] = 255
# image[0:50,0:50,1] = 0
# image[0:50,0:50,2] = 0
# Resize image so that each centimeter is 10 pixels
small_image = imutils.resize(image, width = int(np.ceil(image.shape[1]*2.54/200*pixels_per_cm)))


x_rw, y_rw, z_rw = picture_dict[picture_list[0]]
# top left corner of the first image
tcx = int(np.ceil((np.cos(z_rw)*ix1 - np.sin(z_rw)*iy1 + x_rw - minx)*pixels_per_cm*100))
tcy = int(np.ceil((np.sin(z_rw)*ix1 + np.cos(z_rw)*iy1 + y_rw - miny)*pixels_per_cm*100))

# print full_img.shape, tcx, tcy, width-tcx, (width-tcx-small_image.shape[0]), tcy, (tcy+small_image.shape[1])

# small_image = imutils.rotate_bound(small_image, z_rw*180/np.pi)
# print full_img.shape, small_image.shape 


print "full_img txcy", tcx, tcy

full_img[width+border-tcx:(width+border-tcx+small_image.shape[0]), height+border-tcy:(height+border-tcy+small_image.shape[1])] = small_image
M = cv2.getRotationMatrix2D((width+border-tcx, height+border-tcy), -z_rw*180/np.pi, 1.0)
full_img = cv2.warpAffine(full_img, M, (full_img.shape[1], full_img.shape[0]))

stitcher = Stitcher()
for pic in picture_list:
    image = cv2.imread(directory + pic + ".png")
    small_image = imutils.resize(image, width = int(np.ceil(image.shape[1]*2.54/200*pixels_per_cm)))
    # small_image = imutils.rotate(small_image, picture_dict[pic][2]*180/np.pi)

    x_rw, y_rw, z_rw = picture_dict[pic]
    # top left corner of the first image
    tcx = int(np.ceil((np.cos(z_rw)*ix1 - np.sin(z_rw)*iy1 + x_rw - minx)*pixels_per_cm*100))
    tcy = int(np.ceil((np.sin(z_rw)*ix1 + np.cos(z_rw)*iy1 + y_rw - miny)*pixels_per_cm*100))
    new_img = np.zeros((width,height,3), np.uint8)
    new_img = cv2.copyMakeBorder(new_img, border, border, border, border, cv2.BORDER_CONSTANT, value = [0,0,0])


    print "new_image tcxy", tcx, tcy, small_image.shape

    new_img [width+border-tcx:(width+border-tcx+small_image.shape[0]), height+border-tcy:(height+border-tcy+small_image.shape[1])] = small_image
    M = cv2.getRotationMatrix2D((width+border-tcx, height+border-tcy), -z_rw*180/np.pi, 1.0)
    new_img = cv2.warpAffine(new_img, M, (new_img.shape[1], new_img.shape[0]))


    full_img = stitcher.stitch(full_img, new_img, showMatches=True)
    show = imutils.resize(full_img, height=600)
    cv2.imshow("Result", show)
    # show_vis = imutils.resize(vis, width=600)
    # cv2.imshow("Vis", show_vis)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


show = imutils.resize(full_img, width=600)
cv2.imshow("Final", show)
cv2.waitKey(0)
cv2.destroyAllWindows()


# (result, vis) = stitcher.stitch((small_image, full_img), showMatches=True)
 









