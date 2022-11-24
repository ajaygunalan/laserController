# Mian: https://www.youtube.com/watch?v=hezISpFdxDo
# Other Ref: https://github.com/ndrwnaguib/LaserPointerTracking/blob/master/track_laser.pyimport sys

import cv2
import numpy as np
from scipy import ndimage
from collections import deque


def initFilter(N):
    for i in range(0, N):
        _, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 0, 255])
        upper_red = np.array([255, 255, 255])

        # Checks if array elements lie between lower & upper red.
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Finds the min & max element values and their positions.
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask)
    
        maxLocList = list(maxLoc)
        maxLocList[0] = maxLocList[0] + offset1
        maxLocList[1] = maxLocList[1] + offset2

        qX.append(maxLocList[0])
        qY.append(maxLocList[1])


def lowPassFilter(maxLocList):

    qX.append(maxLocList[0])
    qY.append(maxLocList[1])

    FilterValX = ndimage.median_filter(qX, size=FilterSize)
    FilterValY = ndimage.median_filter(qY, size=FilterSize)

    maxLocList[0] = FilterValX[queueSize-1]
    maxLocList[1] = FilterValY[queueSize-1]

    maxLocList[0] = int(maxLocList[0])
    maxLocList[1] = int(maxLocList[1])

    maxLocTuple =tuple(maxLocList)

    return maxLocTuple


# Parametrs
queueSize = 300
FilterSize = 11
qX = deque(maxlen=queueSize)
qY = deque(maxlen=queueSize)

# Drift offset need to be hand tuned by us.
offset1 = 10
offset2 = 20

input_video_path = "./basicShadow.mp4"
cap = cv2.VideoCapture(input_video_path)

initFilter(queueSize)

while (1):
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 0, 255])
    upper_red = np.array([255, 255, 255])

    # Checks if array elements lie between lower & upper red.
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Finds the min & max element values and their positions.
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask)
    
    maxLocList = list(maxLoc)
    maxLocList[0] = maxLocList[0] + offset1
    maxLocList[1] = maxLocList[1] + offset2

    maxLocTuple = lowPassFilter(maxLocList)

    cv2.circle(frame, maxLocTuple, 20, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.imshow('Track Laser with Filter', frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
