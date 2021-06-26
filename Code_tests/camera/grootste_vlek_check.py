# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import sys
import operator

# om de camera mee te initialiseren
initialized = False

# wordt gebruikt om de randen van de grid te herkennen
pixel_bound_x_low = 640
pixel_bound_x_high = 640
pixel_bound_y_low = 360
pixel_bound_y_high = 360

#----------------------------FUNCTIES----------------------------
# maakt een foto met de camera
def makepicture():
	rawCapture = PiRGBArray(camera)
	# allow the camera to warmup
	time.sleep(0.1)
	# grab an image from the camera
	camera.capture(rawCapture, format="bgr")
	image = rawCapture.array
	cv2.imwrite("newimage.jpg", image)
	#cv2.imshow('Image',image)

# filtert de pixels op basis van kleur in een HSV range
def maskcolor(weak_strength, stronger_strength):
	imagex = cv2.imread("newimage.jpg")

	# Convert BGR to HSV
	hsv = cv2.cvtColor(imagex, cv2.COLOR_BGR2HSV)

	# define color strenght parameters in HSV
	weaker = np.array(weak_strength)
	stronger = np.array(stronger_strength)

	# Threshold the HSV image to obtain input color
	mask = cv2.inRange(hsv, weaker, stronger)

	#cv2.imshow('Result',mask)

	return mask.tolist()

# rekent de gemiddelde uit van de pixels die al gefilterd waren op kleur
def avgpoint(givenmask):
	#print(len(givenmask))
	list_coords = []
	for i in range(len(givenmask)):
		if i > pixel_bound_y_low and i < pixel_bound_y_high: 
	    		for j in range(1280):
				if j > pixel_bound_x_low and j < pixel_bound_x_high:
 	       				if givenmask[i][j] == 255:
 	           				list_coords.append((j, i))

	#temp_x = pixel_bound_x_low
	#temp_y = pixel_bound_y_low
	#while temp_y < pixel_bound_y_high:
	#	while temp_x < pixel_bound_x_high:
	#		#print givenmask[temp_y][temp_x]
 	#		if givenmask[temp_y][temp_x] == 255:
 	#			list_coords.append((temp_x, temp_y))
	#		temp_x += 1
	#	temp_y += 1
	#print list_coords


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

# volgende wordt niet gebruikt maar is om de afstanden tussen twee pixels uit te rekenen
def calc_distance(point1, point2):
	distance = tuple(map(lambda i, j: i - j, point1, point2))
	#schaal berekenen om van pixels naar centimeters om te zetten, en maak absoluut
	return distance

# Wordt gebruikt om de pixel waarde om te zetten naar een grid positie. Dit gebeurt binnen de randen van het veld.
def calc_position(avg_pixel_pos, cells):
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
#----------------------------------------------------------------

while True:

	# Maak de camera klaar voor gebruik en definieer de randen van het veld op basis van een bepaalde kleur op de grond.
	if initialized == False:
		# initialize the camera and grab a reference to the raw camera capture
		camera = PiCamera()
		camera.resolution = (1280, 720)
		initialized = True
		makepicture()
		greenmask = maskcolor([36, 50, 70],[89, 255, 255])
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
		print pixel_bound_y_low
		print pixel_bound_y_high
		print pixel_bound_x_low
		print pixel_bound_x_high
		

	makepicture()

	# definieer kleuren die gefilterd wordt
	bluemask = maskcolor([90, 50, 70], [128, 255, 255])
	greenmask = maskcolor([36, 50, 70], [89, 255, 255])

	#print bluemask

	blue_avg = avgpoint(bluemask)
	green_avg = avgpoint(greenmask)

	print blue_avg
	#print green_avg

	#volgende weghalen
	#blue_green = calc_distance(blue_avg, green_avg)
	#print blue_green

	cells = 10
	blue_grid_pos = calc_position(blue_avg, cells)
	#green_grid_pos = calc_position(green_avg, cells)

	print blue_grid_pos
	#print green_grid_pos

	#time.sleep(0.2)

	#cv2.waitKey(0)
	#cv2.destroyAllWindows()

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