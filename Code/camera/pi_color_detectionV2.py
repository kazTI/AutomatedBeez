
import cv2
import numpy as np
import time as tm
from picamera.array import PiRGBArray
from picamera import PiCamera

import sys
sys.path.append('/home/pi/Desktop/onze_git/AutomatedBeez/Code/')
import lib.services as sv
import lib.credentials as cr

def nothing(x):
    pass

def mouseClicked(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        if y > button[0] and y < button[1] and x > button[2] and x < button[3]:   
            print('Clicked on Button!')

cv2.namedWindow('Color Detection')
cv2.setMouseCallback('Color Detection', mouseClicked)
cv2.createTrackbar("B", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("G", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("R", "Trackbars", 0, 255, nothing)

button = [20, 60, 50, 250]
button_background = np.zeros((80,300), np.uint8)
button_background[button[0]:button[1],button[2]:button[3]] = 180
cv2.putText(button_background, 'Next', (100,50), cv2.FONT_HERSHEY_PLAIN, 2, (0), 3)

cv2.imshow('Control', button_background)
cv2.waitKey(0)
cv2.destroyAllWindows()