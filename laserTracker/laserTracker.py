# Mian: https://www.youtube.com/watch?v=hezISpFdxDo
# Other Ref: https://github.com/ndrwnaguib/LaserPointerTracking/blob/master/track_laser.pyimport sys

import cv2
import numpy as np
from imutils import contours
from scipy.signal import butter,filtfilt

maxLocList = []
maxLocListPrev = []
N = 5


def initFilter(N):
    for i < N:
    ret, frame = cap.read()
    # height, width = frame.shape[:2]
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

    return maxLocList


def lowPassFilter(maxLocList, maxLocListPrev):
    FiltrdVal = [0,0]
    print(maxLocList)
    print(maxLocListPrev)
    gain = 0.8
    FiltrdVal[0] = gain*(maxLocList[0] + maxLocListPrev[0])/2
    FiltrdVal[1] = (maxLocList[1] + maxLocListPrev[0])/2
    return FiltrdVal



input_video_path = "basicShadow.mp4"
# input_video_path = 3
cap = cv2.VideoCapture(input_video_path)

# Drift offset need to be hand tuned by us.
offset1 = 0
offset2 = -100


listOfPrevVal = initFilter(N)

maxLocInt = [0, 0]

while (1):
    ret, frame = cap.read()
    # height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    lower_red = np.array([0, 0, 255])
    upper_red = np.array([255, 255, 255])

    # Checks if array elements lie between lower & upper red.
    mask = cv2.inRange(hsv, lower_red, upper_red)

    detectBlob(mask)
    # cv2.imshow('Mask', mask)

    # Finds the min & max element values and their positions.
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask)
    
    maxLocList = list(maxLoc)
    maxLocList[0] = maxLocList[0] + offset1
    maxLocList[1] = maxLocList[1] + offset2


    maxLocListFiltrd = lowPassFilter(maxLocList, maxLocListPrev)
    maxLocListPrev = maxLocList


    maxLocInt[0] = int(maxLocListFiltrd[0]) 
    maxLocInt[1] = int(maxLocListFiltrd[1]) 

    maxLocTuple =tuple(maxLocInt)
    print(maxLocTuple)
    cv2.circle(frame, maxLocTuple, 20, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.imshow('Track Laser-New', frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
