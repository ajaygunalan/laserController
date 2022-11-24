import numpy as np
import cv2

img = cv2.imread('1.png')

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

blurred = cv2.medianBlur(gray, 25) #cv2.bilateralFilter(gray,10,50,50)

minDist = 100
param1 = 30 #500
param2 = 50 #200 #smaller value-> more false circles
minRadius = 5
maxRadius = 100 #10

# docstring of HoughCircles: HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) -> circles
circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)

# Show result for testing:
cv2.imshow('img', img)
cv2.waitKey(0)
cv2.destroyAllWindows()



# import numpy as np
# import cv2


# input_video_path = "basicShadow.mp4"
# # input_video_path = 3
# cap = cv2.VideoCapture(input_video_path)

# # Drift offset need to be hand tuned by us.
# offset1 = 10
# offset2 = 20

# while (1):
#     ret, img = cap.read()

#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#     blurred = cv2.medianBlur(gray, 25) #cv2.bilateralFilter(gray,10,50,50)

#     minDist = 100
#     param1 = 30 #500
#     param2 = 50 #200 #smaller value-> more false circles
#     minRadius = 5
#     maxRadius = 100 #10

#     # docstring of HoughCircles: HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) -> circles
#     circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

#     if circles is not None:
#         circles = np.uint16(np.around(circles))
#         for i in circles[0,:]:
#             cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)

#     # Show result for testing:
#     cv2.imshow('img', img)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()