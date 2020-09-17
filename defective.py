import cv2
import numpy as np

frame = cv2.imread('defective frame.jpg')
frame = cv2.resize(frame, (640, 360))
print(frame.shape)
gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
#gray_blur = cv2.GaussianBlur(gray,(9, 9),0)
kernel2 = np.ones((10,10),np.uint8)
dilation = cv2.dilate(gray,kernel2,iterations = 3)
edges = cv2.Canny(dilation,10,20,apertureSize = 3)
lines = cv2.HoughLines(edges,1,np.pi/180, 100)
#print(lines)
if lines is not None:
    for r, theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*(r)
        y0 = b*(r)
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        print(x1, y1, x2, y2)
        cv2.line(frame,(x1,y1), (x2,y2), (0,255,255),3)

cv2.imshow('frame', frame)
cv2.imshow('edges', edges)
cv2.imshow('dilation', dilation)
cv2.imshow('gray', gray)
cv2.waitKey(0)
cv2.destroyAllWindows()
