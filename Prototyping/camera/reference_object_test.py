# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from PIL import Image
import math

# initialize the camera and grab a reference to the raw camera capture
#camera = PiCamera()
#rawCapture = PiRGBArray(camera)
# allow the camera to warmup
#time.sleep(0.1)
# grab an image from the camera
#camera.capture(rawCapture, format="bgr")
#image = rawCapture.array
#cv2.imwrite("newimage.jpg", image)

### nieuwe code ###

# Load image and ensure RGB - just in case palettised
im = Image.open("newimage.jpg").convert("RGB")

# Make numpy array from image
npimage=np.array(im)

# Describe what a single red pixel looks like
red=np.array([255,0,0],dtype=np.uint8)

# Find [x,y] coordinates of all red pixels
reds=np.where(np.all((npimage==red),axis=-1))

# Describe what a single blue pixel looks like
blue=np.array([0,0,255],dtype=np.uint8)

# Find [x,y] coordinates of all blue pixels
blues=np.where(np.all((npimage==blue),axis=-1))

print(reds)
print("------------")
print(blues)

testarray = [20, 30, 40, 59, 60]
print(testarray)

#dx2 = (blues[0][0]-reds[0][0])**2          # (200-10)^2
#dy2 = (blues[1][0]-reds[1][0])**2          # (300-20)^2
#distance = math.sqrt(dx2 + dy2)

#print(distance)