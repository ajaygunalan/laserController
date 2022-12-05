
# Mian: https://www.youtube.com/watch?v=hezISpFdxDo
# Other Ref: https://github.com/ndrwnaguib/LaserPointerTracking/blob/master/track_laser.pyimport sys
import cv2
import numpy as np
from scipy import ndimage
from collections import deque

#Gloal Variable
queueSize = 1
x_pre = deque(maxlen=queueSize)
y_pre = deque(maxlen=queueSize)
filterGain = 0.5

def init():
    for i in range(0, queueSize):
        x_pre.append(0)
        y_pre.append(0)

def lowPassFilter(cordi):
    x_filt = int(cordi[0] + (filterGain*x_pre[-1]))
    y_filt = int(cordi[1] + (filterGain*y_pre[-1]))
    return tuple([x_filt, y_filt])


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
