import sys
sys.path.append('/home/pi/Desktop/onze_git/AutomatedBeez/Code/')
import lib.services as sv
import lib.credentials as cr
import time as tm
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import operator
#print(sys.path)

def generateMask(weak_strength, stronger_strength):
    imagex = cv2.imread("masked_image.jpg")

    # convert bgr to hsv and generate mask
    hsv = cv2.cvtColor(imagex, cv2.COLOR_BGR2HSV)
    weaker = np.array(weak_strength)
    stronger = np.array(stronger_strength)
    mask = cv2.inRange(hsv, weaker, stronger)

    cv2.imshow('Masked_Image', mask)

    return mask.tolist()

def getAvarageCoordinates(mask):
    list_coords = []
    for i in range(len(mask)):
        for j in range(1280):
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


clientName = 'pi_camera'
subscribed_topics = [clientName + '_response']
credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient(clientName, subscribed_topics)
mqttClient.startConnection()
tm.sleep(5)

pi_camera = PiCamera()
pi_camera.resolution = (1280, 720)
tm.sleep(1)

greenmask = generateMask([36, 50, 70],[89, 255, 255])
list_coords = []
for i in range(len(greenmask)):
    for j in range(1280):
        if greenmask[i][j] == 255:
            if i < pixel_bound_y_low:
                pixel_bound_y_low = i
            if i > pixel_bound_y_high:
                pixel_bound_y_high = i
            if j < pixel_bound_x_low:
                pixel_bound_x_low = j
            if j > pixel_bound_x_high:
                pixel_bound_x_high = j
print (pixel_bound_y_low)
print (pixel_bound_y_high)
print (pixel_bound_x_low)
print (pixel_bound_x_high)

cells = 10
test = False
running = True
while running:
    image = PiRGBArray(pi_camera)
    pi_camera.capture(image, format="bgr")
    image = image.array
    #cv2.imshow("newimage.jpg", image)

    bluemask = generateMask([90, 50, 70], [128, 255, 255])
    blue_pixel_coordinates = getAvarageCoordinates(bluemask)
    # greenmask = generateMask([36, 50, 70], [89, 255, 255])
    #green_avg = getAvarageCoordinates(greenmask)

    blue_grid_coordinates = calculateGridCoordinates(blue_pixel_coordinates, cells)


    # if test:
    #     cv2.imwrite("original_image.jpg", image)
    #     cv2.imshow('Original Image', image)
    

    x = blue_pixel_coordinates[0]
    y = blue_pixel_coordinates[1]
    image = cv2.circle(image, (x, y), 10, (0, 0, 255), -1)
    cv2.putText(image, f'Coords: {x, y}', (x-60, y-20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 255, 0), 1)
    cv2.imshow('Simulation Window', image)



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