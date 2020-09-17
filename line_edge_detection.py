#!/usr/bin/env python
# license removed for brevity
#import rospy
import cv2 
import numpy as np 
#from geometry_msgs.msg import Twist

def show(img):
    cv2.imshow('FRAME', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main_func():
    counter = 0
    #read from camera
    cap = cv2.VideoCapture('line_latest1.mp4')
    #pub = rospy.Publisher('vel_control_topic', Twist, queue_size=10)
    #rospy.init_node('line_follower_node', anonymous=True)
    #rate = rospy.Rate(20) # 10hz
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")
    
    #Twist control_msg
    # Read until video is completed
    while cap.isOpened(): #and (not rospy.is_shutdown()):
        # Capture frame-by-frame
        if ((counter >= 73) and (counter <= 82)):
            thresh_min = 60
            thresh_max = 70
        elif ((counter >= 83) and (counter <= 86)):
            thresh_min = 50
            thresh_max = 60
        elif ((counter >= 86) and (counter <= 92)):
            thresh_min = 60
            thresh_max = 70
        elif ((counter >= 110) and (counter <= 112)):
            thresh_min = 30
            thresh_max = 50 
        else:
            thresh_min = 20
            thresh_max = 25

        ret, frame = cap.read()
        frame = cv2.resize(frame, (640, 360))
        if ret == True:
            # Convert the img to grayscale 
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            #kernel = 9
            #gray_blur = cv2.GaussianBlur(gray,(kernel, kernel),0)
            kernel = np.ones((17,17),np.uint8)
            erosion = cv2.dilate(gray,kernel,iterations = 3)
            # Apply edge detection method on the image
            edges = cv2.Canny(erosion,thresh_min,thresh_max,apertureSize = 3)
            print(thresh_min, thresh_max)
            # This returns an array of r and theta values  
            lines = cv2.HoughLines(edges,1,np.pi/180, 100)
            #print(lines)
            #print(lines.shape)
            if lines is not None:
                for r,theta in lines[0]:
                    print('check')
                    
                    # Stores the value of cos(theta) in a 
                    a = np.cos(theta) 

                    # Stores the value of sin(theta) in b 
                    b = np.sin(theta) 
                    
                    # x0 stores the value rcos(theta) 
                    x0 = a*r 
                    
                    # y0 stores the value rsin(theta) 
                    y0 = b*r 
                    
                    # x1 stores the rounded off value of (rcos(theta)-1000sin(theta)) 
                    x1 = int(x0 + 1000*(-b)) 
                    
                    # y1 stores the rounded off value of (rsin(theta)+1000cos(theta)) 
                    y1 = int(y0 + 1000*(a)) 

                    # x2 stores the rounded off value of (rcos(theta)+1000sin(theta)) 
                    x2 = int(x0 - 1000*(-b)) 
                    
                    # y2 stores the rounded off value of (rsin(theta)-1000cos(theta)) 
                    y2 = int(y0 - 1000*(a)) 
                                        
                    # cv2.line draws a line in img from the point(x1,y1) to (x2,y2). 
                    # (0,0,255) denotes the colour of the line to be 
                    #drawn. In this case, it is red. 
                    cv2.line(frame,(x1,y1), (x2,y2), (0,0,255),5)

            # Display the resulting frame

            cv2.imshow('Frame',frame)
            cv2.imshow('edges',edges)
            cv2.imshow('erosion',erosion)
            print("frame processed")
        
            # Press Q on keyboard to  exit
            if cv2.waitKey(0) & 0xFF == ord('q'):
                #cv2.imwrite("defective frame.jpg", frame)
                                #print("frame no. : {}".format(counter))
                break
            
            '''control_msg.linear.x =
            control_msg.linear.y =
            control_msg.linear.z =
            control_msg.angular.x =
            control_msg.angular.y =
            control_msg.angular.z =
            
            pub.publish(control_msg)
            '''
            counter += 1
            print(counter)
        else: 
            break
        #rate.sleep()
    

    # When everything done, release the video capture object
    cap.release()
     
    # Closes all the frames
    cv2.destroyAllWindows()
        
main_func()
'''if __name__ == '__main__':
    try:
        main_func()
    except #rospy.ROSInterruptException:
        pass'''
    
    
    

# Reading the required image in 
# which operations are to be done. 
# Make sure that the image is in the same 
# directory in which this python program is 
#img = cv2.imread('7.jpg') 



# The below for loop runs till r and theta values 
# are in the range of the 2d array 
         
    
# All the changes made in the input image are finally 
# written on a new image houghlines.jpg 
#cv2.imwrite('linesDetected.jpg', img) 
