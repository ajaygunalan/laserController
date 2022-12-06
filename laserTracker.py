# Mian: https://www.youtube.com/watch?v=hezISpFdxDo
# Other Ref: https://github.com/ndrwnaguib/LaserPointerTracking/blob/master/track_laser.pyimport sys
import cv2
import numpy as np
from collections import deque

#Gloal Variable
q_size = 60
x_pre = deque(maxlen=q_size)
y_pre = deque(maxlen=q_size)
gain = 0.5


def click_event(event, x, y, flags, params):
	# checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        for i in range(0, q_size):
            x_pre.append(x)
            y_pre.append(y)
    print(x, y)


def init(image):
    cv2.imshow('Select the laser', image)
    cv2.setMouseCallback('Select the laser', click_event)
    cv2.waitKey(0)

def lowPassFilter(cordi):
    x_filt = cordi[0] + (gain * sum(x_pre)/len(x_pre))
    y_filt = cordi[1] + (gain * sum(y_pre)/len(y_pre))

    x_pre.append(x_filt)
    y_pre.append(y_filt)
    return tuple([int(x_filt), int(y_filt)])


def main():
    lower_red = np.array([0, 0, 255])
    upper_red = np.array([255, 255, 255])

    input_video_path = "./data/basicShadow.mp4"
    cap = cv2.VideoCapture(input_video_path)


    is_initialized = True
    while (1):
        _, frame = cap.read()
        if(is_initialized):
            init(frame)
            is_initialized =  False

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
