import cv2
import numpy as np

screen_resolution = [640, 480]
webcam = cv2.VideoCapture(0)
webcam.set(3, screen_resolution[0])
webcam.set(4, screen_resolution[1])

def empty(a):
    pass

cv2.namedWindow('HSV')
cv2.resizeWindow('HSV', 640, 480)
cv2.createTrackbar('HUE Min', 'HSV', 0, 179, empty)
cv2.createTrackbar('HUE Max', 'HSV', 179, 179, empty)
cv2.createTrackbar('SAT Min', 'HSV', 0, 255, empty)
cv2.createTrackbar('SAT Max', 'HSV', 255, 255, empty)
cv2.createTrackbar('VALUE Min', 'HSV', 0, 255, empty)
cv2.createTrackbar('VALUE Max', 'HSV', 255, 255, empty)


while True:
    _, image = webcam.read()

    # image_blur = cv2.GaussianBlur(image, (7, 7), 1)
    # image_gray = cv2.cvtColor(image_blur, cv2.COLOR_BGR2GRAY)
    
    hsv_image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
    hue_min = cv2.getTrackbarPos('HUE Min', 'HSV')
    hue_max = cv2.getTrackbarPos('HUE Max', 'HSV')
    sat_min = cv2.getTrackbarPos('SAT Min', 'HSV')
    sat_max = cv2.getTrackbarPos('SAT Max', 'HSV')
    value_min = cv2.getTrackbarPos('VALUE Min', 'HSV')
    value_max = cv2.getTrackbarPos('VALUE Max', 'HSV')

    lower = np.array([hue_min, sat_min, value_min])
    upper = np.array([hue_max, sat_max, value_max])
    mask = cv2.inRange(hsv_image, lower, upper)
    result = cv2.bitwise_and(image, image, mask = mask)

    cv2.imshow('Original stream', image)
    # cv2.imshow('Gaussian blur', image_blur)
    # cv2.imshow('Grayscale image', image_gray)

    cv2.imshow('HSV stream', hsv_image)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)

    if cv2.waitKey(1) == 27:
        break

webcam.release()
cv2.destroyAllWindows()