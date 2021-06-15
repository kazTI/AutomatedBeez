# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import sys
import operator

#----------------------------FUNCTIES----------------------------
def maskcolor(weak_strength, stronger_strength):
	imagex = cv2.imread("newimage.jpg")

	# Convert BGR to HSV
	hsv = cv2.cvtColor(imagex, cv2.COLOR_BGR2HSV)

	# define color strenght parameters in HSV
	weaker = np.array(weak_strength)
	stronger = np.array(stronger_strength)

	# Threshold the HSV image to obtain input color
	mask = cv2.inRange(hsv, weaker, stronger)

	cv2.imshow('Result',mask)

	return mask.tolist()

def avgpoint(givenmask):
	print(len(givenmask))
	list_coords = []
	for i in range(len(givenmask)):
	    for j in range(1280):
 	       if givenmask[i][j] == 255:
 	           list_coords.append((j, i))

	#print (list_coords)


	xval = 0
	yval = 0

	print len(list_coords)
	for i in range(len(list_coords)):
		xval += list_coords[i][0]
		yval += list_coords[i][1]

	xval = xval / len(list_coords)
	yval = yval / len(list_coords)

	print xval
	print yval
	return xval, yval

#----------------------------------------------------------------

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (1280, 720)
rawCapture = PiRGBArray(camera)
# allow the camera to warmup
time.sleep(0.1)
# grab an image from the camera
camera.capture(rawCapture, format="bgr")
image = rawCapture.array
cv2.imwrite("newimage.jpg", image)
cv2.imshow('Image',image)

# definieer kleuren die gefilterd wordt
bluemask = maskcolor([90, 50, 70], [128, 255, 255])

#print bluemask

blue_avg = avgpoint(bluemask)
print blue_avg

cv2.waitKey(0)
cv2.destroyAllWindows()

#color_dict_HSV = {'black': [[180, 255, 30], [0, 0, 0]],
#              'white': [[180, 18, 255], [0, 0, 231]],
#              'red1': [[180, 255, 255], [159, 50, 70]],
#              'red2': [[9, 255, 255], [0, 50, 70]],
#              'green': [[89, 255, 255], [36, 50, 70]],
#              'blue': [[128, 255, 255], [90, 50, 70]],
#              'yellow': [[35, 255, 255], [25, 50, 70]],
#              'purple': [[158, 255, 255], [129, 50, 70]],
#              'orange': [[24, 255, 255], [10, 50, 70]],
#              'gray': [[180, 18, 230], [0, 0, 40]]}

# https://programmingdesignsystems.com/color/color-models-and-color-spaces/index.html#:~:text=HSV%20is%20a%20cylindrical%20color,easier%20for%20humans%20to%20understand.&text=Hue%20specifies%20the%20angle%20of,the%20amount%20of%20color%20used.
# kijk naar HSV