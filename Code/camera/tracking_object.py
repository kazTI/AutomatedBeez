import cv2

def createBBox():
    tracker = cv2.TrackerCSRT_create()

def drawBox(image, bbox, color):
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    cv2.rectangle(image, (x, y), ((x + w), (y + h)), color, 2, 1)

def getCenterCoords(bbox):
    x = bbox[2] / 2 + bbox[0]
    y = bbox[3] / 2 + bbox[1]
    return (int(x), int(y))

cap = cv2.VideoCapture(0)
success, image = cap.read()

# tracker = cv2.TrackerMOSSE_create()
tracker = cv2.TrackerCSRT_create()
bbox_drone_1 = cv2.selectROI('Drone_1', image, False)
tracker.init(image, bbox_drone_1)
color_drone_1 = (255, 165, 0)

tracker2 = cv2.TrackerCSRT_create()
bbox_drone_2 = cv2.selectROI('Drone_2', image, False)
tracker2.init(image, bbox_drone_2)
color_drone_2 = (34, 139, 34)

while True:
    timer = cv2.getTickCount()
    success, image = cap.read()

    success_drone_1, bbox_drone_1 = tracker.update(image)
    success_drone_2, bbox_drone_2 = tracker2.update(image)
    print(bbox_drone_1)

    if success_drone_1:
        x, y = getCenterCoords(bbox_drone_1)
        drawBox(image, bbox_drone_1, color_drone_1)
        cv2.putText(image, 'Tracking Drone 1', (10, 50), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(image, f'Coords: {x, y}', (x-60, y-20), cv2.FONT_HERSHEY_DUPLEX, 0.4, color_drone_1, 1)
        cv2.circle(image, (x, y), 10, color_drone_1, -1)
    else:
        cv2.putText(image, 'Lost Drone 1', (10, 50), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 0, 255), 1)
        
    if success_drone_2:
        x, y = getCenterCoords(bbox_drone_2)
        drawBox(image, bbox_drone_2, color_drone_2)
        cv2.putText(image, 'Tracking Drone 2', (10, 70), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(image, f'Coords: {x, y}', (x-60, y-20), cv2.FONT_HERSHEY_DUPLEX, 0.4, color_drone_2, 1)
        cv2.circle(image, (x, y), 10, color_drone_2, -1)
    else:
        cv2.putText(image, 'Lost Drone 2', (10, 70), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 0, 255), 1)

    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    cv2.putText(image, str(int(fps)), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 0, 255), 1)
    cv2.imshow('Life View', image)

    if cv2.waitKey(1) == 27:
        break