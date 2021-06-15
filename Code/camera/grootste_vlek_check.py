# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import sys
import operator

initialized = False

pixel_bound_x_low = 640
pixel_bound_x_high = 640
pixel_bound_y_low = 360
pixel_bound_y_high = 360

#----------------------------FUNCTIES----------------------------
def makepicture():
	rawCapture = PiRGBArray(camera)
	# allow the camera to warmup
	time.sleep(0.1)
	# grab an image from the camera
	camera.capture(rawCapture, format="bgr")
	image = rawCapture.array
	cv2.imwrite("newimage.jpg", image)
	cv2.imshow('Image',image)

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
	#print(len(givenmask))
	list_coords = []
	for i in range(len(givenmask)):
	    for j in range(1280):
 	       if givenmask[i][j] == 255:
 	           list_coords.append((j, i))

	#print (list_coords)


	xval = 0
	yval = 0

	#print len(list_coords)

	if len(list_coords) != 0:
		for i in range(len(list_coords)):
			xval += list_coords[i][0]
			yval += list_coords[i][1]

		xval = xval / len(list_coords)
		yval = yval / len(list_coords)

		#print xval
		#print yval

	return xval, yval

#volgende wordt wss niet gebruikt
def calc_distance(point1, point2):
	distance = tuple(map(lambda i, j: i - j, point1, point2))
	#schaal berekenen om van pixels naar centimeters om te zetten, en maak absoluut
	return distance


def calc_position(avg_pixel_pos):
	leftSpanX = pixel_bound_x_high - pixel_bound_x_low
	rightSpanX = 29 - 0

	valueScaledX = float(avg_pixel_pos[0] - pixel_bound_x_low) / float(leftSpanX)

	converted_x = 0 + (valueScaledX * rightSpanX)

	leftSpanY = pixel_bound_y_high - pixel_bound_y_low
	rightSpanY = 29 - 0

	valueScaledY = float(avg_pixel_pos[0] - pixel_bound_y_low) / float(leftSpanY)

	converted_y = 0 + (valueScaledY * rightSpanY)

	converted_pos = (round(converted_x), round(converted_y))

	return converted_pos
#----------------------------------------------------------------

while True:

	if initialized == False:
		# initialize the camera and grab a reference to the raw camera capture
		camera = PiCamera()
		camera.resolution = (1280, 720)
		initialized = True
		makepicture()
		graymask = maskcolor([0, 0, 40],[180, 18, 230])
		list_coords = []
		for i in range(len(graymask)):
			for j in range(1280):
 				if graymask[i][j] == 255:
					if i < pixel_bound_y_low:
						pixel_bound_y_low = i
					if i > pixel_bound_y_high:
						pixel_bound_y_high = i
					if j < pixel_bound_x_low:
						pixel_bound_x_low = j
					if j > pixel_bound_x_high:
						pixel_bound_x_high = j
		print pixel_bound_y_low
		print pixel_bound_y_high
		print pixel_bound_x_low
		print pixel_bound_x_high
		

	makepicture()

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
	greenmask = maskcolor([36, 50, 70], [89, 255, 255])

	#print bluemask

	blue_avg = avgpoint(bluemask)
	green_avg = avgpoint(greenmask)

	print blue_avg
	print green_avg

	#volgende weghalen
	#blue_green = calc_distance(blue_avg, green_avg)
	#print blue_green

	blue_grid_pos = calc_position(blue_avg)
	green_grid_pos = calc_position(green_avg)

	print blue_grid_pos
	print green_grid_pos

	time.sleep(0.2)

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