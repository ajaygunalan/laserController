# Mian: https://www.youtube.com/watch?v=hezISpFdxDo
# Other Ref: https://github.com/ndrwnaguib/LaserPointerTracking/blob/master/track_laser.pyimport sys

import cv2
import numpy as np
from scipy import ndimage
from collections import deque


def getCentre():
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Checks if array elements lie between lower & upper red.
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Find all the contours in the binary mask
    contours,hierarchy = cv2.findContours(mask,2,1)

    big_contour = []
    max = 0
    cX = 0
    cY = 0

    for i in contours:
        area = cv2.contourArea(i) #--- find the contour having biggest area ---
        if(area > max):
            max = area
            big_contour = i
            # compute the center of the contour
            M = cv2.moments(big_contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

    return [cX, cY], frame, big_contour, mask


def initFilter(N):
    for i in range(0, N):
        centre, _, _, _ = getCentre()
        qX.append(centre[0])
        qY.append(centre[1])


def lowPassFilter(center):

    qX.append(center[0])
    qY.append(center[1])

    FilterValX = ndimage.median_filter(qX, size=FilterSize)
    FilterValY = ndimage.median_filter(qY, size=FilterSize)

    center[0] = sum(FilterValX)/len(FilterValX)
    center[1] = sum(FilterValY)/len(FilterValY)

    center[0] = int(center[0])
    center[1] = int(center[1])

    return tuple(center)


# Parametrs
queueSize = 60
FilterSize = 5
qX = deque(maxlen=queueSize)
qY = deque(maxlen=queueSize)

# Drift offset need to be hand tuned by us.
offset1 = 10
offset2 = 20

lower_red = np.array([0, 0, 255])
upper_red = np.array([255, 255, 255])

input_video_path = "./data/basicShadow.mp4"
# input_video_path = 4
cap = cv2.VideoCapture(input_video_path)

initFilter(queueSize) # 1 SOR 2SECONDS
while (1):
    [x, y], frame, contour, mask = getCentre()
    (xFilt, yFilt) = lowPassFilter([x,y])

    frameFilter = frame
    cv2.circle(mask, tuple([x,y]), 20, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.drawContours(mask, contour, -1, (0,255,0), 3)

    cv2.circle(frameFilter, (xFilt, yFilt) , 20, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.drawContours(frameFilter, contour, -1, (0,255,0), 3)

    cv2.imshow('mask', mask)
    cv2.imshow('Filterd', frameFilter)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
