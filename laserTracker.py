
# Mian: https://www.youtube.com/watch?v=hezISpFdxDo
# Other Ref: https://github.com/ndrwnaguib/LaserPointerTracking/blob/master/track_laser.pyimport sys
import cv2
import numpy as np
from scipy import ndimage
from collections import deque

#Gloal Variable
queueSize = 10
qX = deque(maxlen=queueSize)
qY = deque(maxlen=queueSize)

def init():
    for i in (0, queueSize):
        qX.append(i)
        qY.append(i)

def lowPassFilter(center):
    qX.append(center[0])
    qY.append(center[1])

    FilterValX = ndimage.median_filter(qX, size=5)
    FilterValY = ndimage.median_filter(qY, size=5)

    center[0] = sum(FilterValX)/len(FilterValX)
    center[1] = sum(FilterValY)/len(FilterValY)

    center[0] = int(center[0])
    center[1] = int(center[1])

    return tuple(center)


def main():
    lower_red = np.array([0, 0, 255])
    upper_red = np.array([255, 255, 255])

    input_video_path = "./data/basicShadow.mp4"
    cap = cv2.VideoCapture(input_video_path)

    init()

    while (1):
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

        (xFilt, yFilt) = lowPassFilter([cX,cY])

        frameFilter = frame
        cv2.circle(mask, tuple([cX,cY]), 20, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.drawContours(mask, big_contour, -1, (0,255,0), 3)

        cv2.circle(frameFilter, (xFilt, yFilt) , 20, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.drawContours(frameFilter, big_contour, -1, (0,255,0), 3)

        cv2.imshow('mask', mask)
        cv2.imshow('Filterd', frameFilter)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()

if __name__=="__main__":
    main()
