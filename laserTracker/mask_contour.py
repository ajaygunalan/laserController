import cv2
import numpy as np
from imutils import contours

# Read image.
img = cv2.imread('/home/sli/calm_ws/laser_start_mask.png', cv2.IMREAD_COLOR)
  
# Convert to grayscale.
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#--- First obtain the threshold using the greyscale image ---
ret,th = cv2.threshold(gray,127,255, 0)

#--- Find all the contours in the binary image ---
contours,hierarchy = cv2.findContours(th,2,1)
cnt = contours
big_contour = []
max = 0
for i in cnt:
    area = cv2.contourArea(i) #--- find the contour having biggest area ---
    if(area > max):
        max = area
        big_contour = i 

final = cv2.drawContours(img, big_contour, -1, (0,255,0), 3)
cv2.imshow('final', final)
cv2.imwrite("laser_start_mask_contour.png", final)