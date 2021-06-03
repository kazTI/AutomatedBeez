# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
rawCapture = PiRGBArray(camera)
# allow the camera to warmup
time.sleep(0.1)
# grab an image from the camera
camera.capture(rawCapture, format="bgr")
image = rawCapture.array
cv2.imwrite("newimage.jpg", image)

# image = cv2.imread("coloredchips.png")
imagex = cv2.imread("newimage.jpg")

# Convert BGR to HSV
hsv = cv2.cvtColor(imagex, cv2.COLOR_BGR2HSV)


# define color strenght parameters in HSV
weaker = np.array([110, 100, 100])
stronger = np.array([130,255,255])

lower_blue = np.array([90,50,70])
upper_blue = np.array([128,255,255])

# Threshold the HSV image to obtain input color
mask = cv2.inRange(hsv, weaker, stronger)

# Threshold the HSV image to obtain input color (blue)
bluemask = cv2.inRange(hsv, lower_blue, upper_blue)

cv2.imshow('Image',image)
cv2.imshow('Result',mask)
cv2.imshow('ResultBlue',bluemask)

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