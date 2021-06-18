
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
            masks.append(mask)

            
def generateMask(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    B = cv2.getTrackbarPos('B', 'Color Calibration')
    G = cv2.getTrackbarPos('G', 'Color Calibration')
    R = cv2.getTrackbarPos('R', 'Color Calibration')
    
    rgb_color = np.uint8([[[B, G, R]]])
    hsv_color = cv2.cvtColor(rgb_color ,cv2.COLOR_BGR2HSV)
    lowerLimit = np.uint8([hsv_color[0][0][0]-10,100,100])
    upperLimit = np.uint8([hsv_color[0][0][0]+10,255,255])

    mask = cv2.inRange(hsv, lowerLimit, upperLimit)
    
    cv2.imshow('Mask', mask)
    
    return mask




# initialize the camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))


# initialize window for calibration
masks = []
cv2.namedWindow('Color Calibration')
cv2.setMouseCallback('Color Calibration', mouseClicked)
cv2.createTrackbar('B', 'Color Calibration', 0, 255, nothing)
cv2.createTrackbar('G', 'Color Calibration', 0, 255, nothing)
cv2.createTrackbar('R', 'Color Calibration', 0, 255, nothing)


button = [20, 60, 50, 250]
button_background = np.zeros((80,300), np.uint8)
button_background[button[0]:button[1],button[2]:button[3]] = 180
cv2.putText(button_background, 'Next', (100,50), cv2.FONT_HERSHEY_PLAIN, 2, (0), 3)
cv2.imshow('Color Calibration', button_background)

global mask
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    mask = generateMask(image)
    result = cv2.bitwise_and(image, image , mask=mask)
    
    cv2.imshow("Camera stream", image)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)

    rawCapture.truncate(0)
    if cv2.waitKey(1) == 27:
        cv2.destroyAllWindows()
        break

print('closing')