# Mian: https://www.youtube.com/watch?v=hezISpFdxDo
# Other Ref: https://github.com/ndrwnaguib/LaserPointerTracking/blob/master/track_laser.pyimport sys

import cv2
import numpy as np
from scipy import ndimage
from collections import deque

import matplotlib.pyplot as plt

from skimage import data, color, img_as_ubyte
from skimage.feature import canny
from skimage.transform import hough_ellipse
from skimage.draw import ellipse_perimeter

def getCentre():
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Checks if array elements lie between lower & upper red.
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # # Perform a Hough Transform
    # # The accuracy corresponds to the bin size of a major axis.
    # # The value is chosen in order to get a single high accumulator.
    # # The threshold eliminates low accumulators
    # result = hough_ellipse(mask, accuracy=20, threshold=250,
    #                    min_size=100, max_size=120)
    # result.sort(order='accumulator')
    #
    # # Estimated parameters for the ellipse
    # best = list(result[-1])
    # yc, xc, a, b = (int(round(x)) for x in best[1:5])
    # orientation = best[5]

    # # Draw the ellipse on the original image
    # cy, cx = ellipse_perimeter(yc, xc, a, b, orientation)
    #
    # # Draw the ellipse on the original image
    # cy, cx = ellipse_perimeter(yc, xc, a, b, orientation)


    # Finds the min & max element values and their positions.
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask)

    maxLoc= list(maxLoc)
    maxLoc[0] = maxLoc[0] + offset1
    maxLoc[1] = maxLoc[1] + offset2

    # return [cy, cx], frame
    return maxLoc, frame


def initFilter(N):
    for i in range(0, N):
        centre, _ = getCentre()
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

# input_video_path = "./basicShadow.mp4"
input_video_path = 4
cap = cv2.VideoCapture(input_video_path)

initFilter(queueSize) # 1 SOR 2SECONDS
while (1):
    [x, y], frame = getCentre()
    (xFilt, yFilt) = lowPassFilter([x,y])

    frameFilter = frame.copy()
    cv2.circle(frame, tuple([x,y]), 20, (0, 0, 255), 2, cv2.LINE_AA)


    # frameFilter[yFilt, xFilt] = (0, 0, 255)
    cv2.circle(frameFilter, (xFilt, yFilt) , 20, (0, 0, 255), 2, cv2.LINE_AA)

    horiz = np.concatenate((frame, frameFilter), axis=1)

    cv2.imshow('Original and Filterd', horiz)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
