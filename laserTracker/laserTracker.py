# Mian: https://www.youtube.com/watch?v=hezISpFdxDo
# Other Ref: https://github.com/ndrwnaguib/LaserPointerTracking/blob/master/track_laser.pyimport sys

import cv2
import numpy as np
from imutils import contours
input_video_path = "basicShadow.mp4"
# input_video_path = 3
cap = cv2.VideoCapture(input_video_path)
pts = []

# Drift offset need to be hand tuned by us.
offset1 = 10
offset2 = 20

# We need to set resolutions.
# so, convert them from float to integer.
# frame_width = int(cap.get(3))
# frame_height = int(cap.get(4))
# size = (frame_width, frame_height)


# Below VideoWriter object will create
# a frame of above defined The output
# is stored in 'filename.avi' file.
# result = cv2.VideoWriter('filename.avi',cv2.VideoWriter_fourcc(*'MJPG'), 10, size)
while (1):
    ret, frame = cap.read()
    # height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    lower_red = np.array([0, 0, 255])
    upper_red = np.array([255, 255, 255])

    # Checks if array elements lie between lower & upper red.
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Finds the min & max element values and their positions.
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask)
    maxLocList = []
    maxLocList = list(maxLoc)
    maxLocList[0] = maxLocList[0] + offset1
    maxLocList[1] = maxLocList[1] + offset2
    maxLocTuple =tuple(maxLocList)
    cv2.circle(frame, maxLocTuple, 20, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.imshow('Track Laser', frame)

    # Write the frame into the
    # file 'filename.avi'
    # result.write(frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
