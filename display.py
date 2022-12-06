import numpy as np
import cv2
input_video_path = "./data/basicShadow.mp4"
cap = cv2.VideoCapture(input_video_path)
point = (0, 0)
def click(event, x, y, flags, param):
  global point, pressed
  if event == cv2.EVENT_LBUTTONDOWN:
    print("Pressed",x,y)
    point = (x,y)
cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame",click)


while(True):
  ret, frame = cap.read()
  cv2.circle(frame, point, 20, (0, 0, 255), 2, cv2.LINE_AA)
  cv2.imshow("Frame",frame)
  ch = cv2.waitKey(1)
  if ch & 0xFF == ord('q'):
    break
cap.release()
cv2.destroyAllWindows()
