import cv2
import numpy as np

cap = cv2.VideoCapture('line_latest1.mp4')
counter = 0
if (cap.isOpened()== False): 
    print("Error opening video stream or file")

while cap.isOpened():
    ret, frame = cap.read()
    if ret == True:
        cv2.imshow('Frame',frame)
        
        if cv2.waitKey(0) & 0xFF == ord('q'):
            cv2.imwrite("defective frame.jpg", frame)
            break
        counter += 1
        print(counter)
    else:
        break
    

cap.release()
cv2.destroyAllWindows()
