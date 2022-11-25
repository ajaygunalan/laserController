import matplotlib.pyplot as plt
import cv2
from skimage import data, color, img_as_ubyte
from skimage.feature import canny
from skimage.transform import hough_ellipse
from skimage.draw import ellipse_perimeter
import numpy as np

lower_red = np.array([0, 0, 255])
upper_red = np.array([255, 255, 255])


input_video_path = "./basicShadow.mp4"
cap = cv2.VideoCapture(input_video_path)
while(1):
    _, frame = cap.read()
    image_gray = color.rgb2gray(frame)
    edges = canny(image_gray, sigma=2.0, low_threshold=0.55, high_threshold=0.8)

    # Perform a Hough Transform
    # The accuracy corresponds to the bin size of a major axis.
    # The value is chosen in order to get a single high accumulator.
    # The threshold eliminates low accumulators
    result = hough_ellipse(edges, accuracy=1, threshold=1, min_size=1, max_size=120)

    if result.size > 0:
        print(result)
        result.sort(order='accumulator')

        # Estimated parameters for the ellipse
        best = list(result[-1])
        yc, xc, a, b = (int(round(x)) for x in best[1:5])
        orientation = best[5]

        # Draw the ellipse on the original image
        cy, cx = ellipse_perimeter(yc, xc, a, b, orientation)
        frame[cy, cx] = (0, 0, 255)
        # Draw the edge (white) and the resulting ellipse (red)
        edges = color.gray2rgb(img_as_ubyte(edges))
        edges[cy, cx] = (250, 0, 0)

        horiz = np.concatenate((frame, edges), axis=1)

        cv2.imshow('Original and Filterd', horiz)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


cap.release()

