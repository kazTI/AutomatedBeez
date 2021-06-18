
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
            print(mask)
            print(' ')

def generateMask(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    B = cv2.getTrackbarPos('B', 'Color Calibration')
    G = cv2.getTrackbarPos('G', 'Color Calibration')
    R = cv2.getTrackbarPos('R', 'Color Calibration')
    
    rgb_color = np.uint8([[[B, G, R]]])
    hsv_color = cv2.cvtColor(rgb_color ,cv2.COLOR_BGR2HSV)
    
    lowerLimit = np.uint8([hsv_color[0][0][0]-10,100,100])
    print(lowerLimit)
    upperLimit = np.uint8([hsv_color[0][0][0]+10,255,255])
    print(upperLimit)
    
    mask = cv2.inRange(hsv, lowerLimit, upperLimit)
    
    cv2.imshow('Mask', mask)
    
    return mask

def getAvarageCoordinates(mask):
    list_coords = []
    for i in range(len(mask)):
        if i > pixel_bound_y_low and i < pixel_bound_y_high: 
            for j in range(1280):
                if j > pixel_bound_x_low and j < pixel_bound_x_high:
                    if mask[i][j] == 255:
                        list_coords.append((j, i))
    #print (list_coords)

    xval = 0
    yval = 0
    for i in range(len(list_coords)):
        xval += list_coords[i][0]
        yval += list_coords[i][1]

    xval = xval / len(list_coords)
    yval = yval / len(list_coords)
    return xval, yval

def calculateGridCoordinates(avg_pixel_pos, cells):
	#deltaX = pixel_bound_x_high - pixel_bound_x_low
	#gridX = float((avg_pixel_pos[0] - pixel_bound_x_low)) / float((deltaX)) * cells

	#deltaY = pixel_bound_y_high - pixel_bound_y_low
	#gridY = float((avg_pixel_pos[1] - pixel_bound_y_low)) / float((deltaY)) * cells
	
	leftSpanX = pixel_bound_x_high - pixel_bound_x_low
	rightSpanX = 9 - 0

	valueScaledX = float(avg_pixel_pos[0] - pixel_bound_x_low) / float(leftSpanX)

	converted_x = 0 + (valueScaledX * rightSpanX)

	leftSpanY = pixel_bound_y_high - pixel_bound_y_low
	rightSpanY = 9 - 0

	valueScaledY = float(avg_pixel_pos[1] - pixel_bound_y_low) / float(leftSpanY)

	converted_y = 0 + (valueScaledY * rightSpanY)

	converted_pos = (round(converted_x), round(converted_y))

	#return (gridX, gridY)
	return converted_pos




# initialize mqtt client
clientName = 'pi_camera'
credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient(clientName)
mqttClient.startConnection()

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
    green_mask = generateMask(image)
    result = cv2.bitwise_and(image, image , mask=green_mask)
    
    cv2.imshow("Camera stream", image)
    cv2.imshow("Mask", green_mask)
    cv2.imshow("Result", result)

    rawCapture.truncate(0)
    if cv2.waitKey(1) == 27:
        cv2.destroyWindow('Camera stream')
        cv2.destroyWindow('Mask')
        cv2.destroyWindow('Result')
        break

pixel_bound_x_low = 640
pixel_bound_x_high = 640
pixel_bound_y_low = 360
pixel_bound_y_high = 360
list_coords = []
for i in range(len(green_mask)):
    for j in range(1280):
        if green_mask[i][j] == 255:
            if i < pixel_bound_y_low:
                pixel_bound_y_low = i
            if i > pixel_bound_y_high:
                pixel_bound_y_high = i
            if j < pixel_bound_x_low:
                pixel_bound_x_low = j
            if j > pixel_bound_x_high:
                pixel_bound_x_high = j
print(f'Grid resolution: ')
print('    x_range: ', (pixel_bound_x_low, pixel_bound_x_high))
print('    y_range: ', (pixel_bound_y_low, pixel_bound_y_high))

cells = 10
# execute main detection
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = image.array
    
    blue_mask = generateMask(image)
    blue_pixel_coordinates = getAvarageCoordinates(blue_mask)
    print('Pixel coords: ', blue_pixel_coordinates)

    blue_grid_coordinates = calculateGridCoordinates(blue_pixel_coordinates, cells)
    print('Grid coords: ', blue_grid_coordinates)

    image = cv2.circle(image, (blue_pixel_coordinates[0], blue_pixel_coordinates[1]), 10, (0, 0, 255), -1)
    cv2.putText(image, f'Coords: {blue_pixel_coordinates[0], blue_pixel_coordinates[1]}', 
                        (blue_pixel_coordinates[0]-60, blue_pixel_coordinates[1]-20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 255, 0), 1)

    cv2.imshow('Simulation Window', image)
    cv2.imshow("Mask", blue_mask)