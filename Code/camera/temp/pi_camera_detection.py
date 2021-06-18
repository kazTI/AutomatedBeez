#import lib.services as sv
#import lib.credentials as cr
import time as tm
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import sys
import operator

def generateMask(weak_strength, stronger_strength):
    imagex = cv2.imread("masked_image.jpg")

    # convert bgr to hsv and generate mask
    hsv = cv2.cvtColor(imagex, cv2.COLOR_BGR2HSV)
    weaker = np.array(weak_strength)
    stronger = np.array(stronger_strength)
    mask = cv2.inRange(hsv, weaker, stronger)

    cv2.imshow('Masked_Image', mask)

    return mask.tolist()

def getCoordinates(mask):
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

test = False
running = True
while running:
    image = PiRGBArray(pi_camera)
    pi_camera.capture(image, format="bgr")
    image = image.array

    if test:
        cv2.imwrite("original_image.jpg", image)
        cv2.imshow('Original Image', image)

    blue_mask = generateMask([90, 200, 70], [128, 255, 255])
    green_mask = generateMask([36, 50, 70], [89, 255, 255])
    
    blue_coordinates = getCoordinates(blue_mask)
    print(blue_coordinates)

    x = blue_coordinates[0]
    y = blue_coordinates[1]
    image = cv2.circle(image, (x, y), 10, (0, 0, 255), -1)
    cv2.putText(image, f'Coords: {x, y}', (x-60, y-20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 255, 0), 1)

