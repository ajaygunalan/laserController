import cv2
input_video_path = "data/basicShadow.mp4"
vid = cv2.VideoCapture(input_video_path)
while(True):
    ret, frame = vid.read()
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
