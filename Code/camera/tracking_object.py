import cv2
import sys
import os
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
from lib import services as sv
from lib import credentials as cr
import time as tm

def click_event(event, x, y, flags, image):
    global pixel_bound_x_low
    global pixel_bound_x_high
    global pixel_bound_y_low
    global pixel_bound_y_high
    global food_location
    global food_location_in_pixel

    if event == cv2.EVENT_LBUTTONDOWN:
        global click
        if click == 0:
            text = f'Pixel_bound_x_low: {x}'
            pixel_bound_x_low = x
            cv2.circle(image, (x, y), 5, (255, 255, 255), -1)
            cv2.putText(image, text, (x-60, y-20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 255, 0), 1)
        elif click == 1:
            text = f'Pixel_bound_x_high: {x}'
            pixel_bound_x_high = x
            cv2.circle(image, (x, y), 5, (255, 255, 255), -1)
            cv2.putText(image, text, (x-60, y-20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 255, 0), 1)
        elif click == 2:
            text = f'Pixel_bound_y_low: {y}'
            pixel_bound_y_low = y
            cv2.circle(image, (x, y), 5, (255, 255, 255), -1)
            cv2.putText(image, text, (x-60, y-20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 255, 0), 1)
        elif click == 3:
            text = f'Pixel_bound_y_high: {y}'
            pixel_bound_y_high = y
            cv2.circle(image, (x, y), 5, (255, 255, 255), -1)
            cv2.putText(image, text, (x-60, y-20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 255, 0), 1)
        elif click == 4:
            food_location_in_pixel = (x, y)
            cv2.circle(image, food_location_in_pixel, 50, (0, 0, 255), 2, cv2.LINE_4)
            food_location = (calculateGridCoordinates(food_location_in_pixel, cells))
        
        cv2.imshow('Configuration', image)
        click += 1
    elif event == cv2.EVENT_RBUTTONDOWN:
        cv2.destroyWindow('Configuration')


def drawBox(image, bbox, color):
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    cv2.rectangle(image, (x, y), ((x + w), (y + h)), color, 2, 1)

def getCenterCoords(bbox):
    x = bbox[2] / 2 + bbox[0]
    y = bbox[3] / 2 + bbox[1]
    return (int(x), int(y))

def calculateGridCoordinates(pixel_coords, cells):
    deltaX = pixel_bound_x_high - pixel_bound_x_low
    gridX = float((pixel_coords[0] - pixel_bound_x_low)) / float((deltaX)) * cells

    deltaY = pixel_bound_y_high - pixel_bound_y_low
    gridY = float((pixel_coords[1] - pixel_bound_y_low)) / float((deltaY)) * cells
    return (int(gridX), int(gridY))

def createMessageDrone(drone1_position=None, drone2_position=None):
    message =   {
                    "crazyflie_0": drone1_position,
                    "crazyflie_1": drone2_position,
                }
    return message

def createMessageFood(food_locations_list=[]):
    message =   {
                    "food": food_locations_list
                }
    return message

global click
click = 0
cells = 10
pixel_bound_x_low = 0
pixel_bound_x_high = 0
pixel_bound_y_low = 0
pixel_bound_y_high = 0

food_location = []

second_drone = False

width = 1280
height = 720
cap = cv2.VideoCapture(0)
cap.set(3, width)
cap.set(4, height)
success, image = cap.read()

cv2.imshow('Configuration', image)
cv2.setMouseCallback('Configuration', click_event, image)

# tracker = cv2.TrackerMOSSE_create()
tracker = cv2.TrackerCSRT_create()
bbox_drone_1 = cv2.selectROI('Drone_1', image, False)
tracker.init(image, bbox_drone_1)
color_drone_1 = (255, 165, 0)

tracker2 = cv2.TrackerCSRT_create()
bbox_drone_2 = cv2.selectROI('Drone_2', image, False)
if second_drone:
    print(bbox_drone_2)
    tracker2.init(image, bbox_drone_2)
color_drone_2 = (34, 139, 34)

# initialize mqtt client for camera
clientName = 'camera'
credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient(clientName, [])
mqttClient.startConnection()

time = tm.time()

time_passed = 0
response_time = 0.2

while True:
    timer = cv2.getTickCount()
    success, image = cap.read()

    success_drone_1, bbox_drone_1 = tracker.update(image)
    success_drone_2, bbox_drone_2 = tracker2.update(image)
    # print(bbox_drone_1)

    if success_drone_1:
        x1, y1 = getCenterCoords(bbox_drone_1)
        drawBox(image, bbox_drone_1, color_drone_1)
        cv2.putText(image, 'Tracking Drone 1', (10, 50), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(image, f'Coords: {x1, y1}', (x1-60, y1-20), cv2.FONT_HERSHEY_DUPLEX, 0.4, color_drone_1, 1)
        cv2.circle(image, (x1, y1), 10, color_drone_1, -1)
    else:
        cv2.putText(image, 'Lost Drone 1', (10, 50), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 0, 255), 1)
        
    if success_drone_2:
        x2, y2 = getCenterCoords(bbox_drone_2)
        drawBox(image, bbox_drone_2, color_drone_2)
        cv2.putText(image, 'Tracking Drone 2', (10, 70), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(image, f'Coords: {x2, y2}', (x2-60, y2-20), cv2.FONT_HERSHEY_DUPLEX, 0.4, color_drone_2, 1)
        cv2.circle(image, (x2, y2), 10, color_drone_2, -1)
    else:
        cv2.putText(image, 'Lost Drone 2', (10, 70), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 0, 255), 1)

    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    cv2.putText(image, str(int(fps)), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 0, 255), 1)
    cv2.circle(image, food_location_in_pixel, 50, (0, 0, 255), 2, cv2.LINE_4)
    cv2.imshow('Life View', image)

    # this code is for sending the calculated grid coordinates to the server
    old_time = time
    time = tm.time()
    time_passed += time - old_time
    if time_passed > response_time:
        print('Coordinates:')
        print('    - pixel coords drone 1: ', (x1, y1))
        mqttClient.sendPublish('real_drone_instructions', ('movement', [x1, y1]), 0)
        x1, y1 = calculateGridCoordinates((x1, y1), cells)
        print('    - grid coords drone 1: ', (x1, y1))

        print('Food Location:')
        print('    - grid coords: ', food_location)

        if second_drone:
            print('Pixel Coords Drone 2: ', (x2, y2))
            x2, y2 = calculateGridCoordinates((x2, y2), cells)
            print('Grid Coords Drone 2: ', (x2, y2))

        message_drone = createMessageDrone([x1, y1])
        message_food = createMessageFood(food_location)
        mqttClient.sendPublish('drone', message_drone, 0)
        mqttClient.sendPublish('food', message_food, 0)
        time_passed = 0
        print(' ')

    if cv2.waitKey(1) == 27:
        break
