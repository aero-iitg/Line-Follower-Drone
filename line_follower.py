import rospy
import cv2
import numpy as np
import math
import time
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *


def setArm():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException:
        print("Service arming call failed")

def setOffboardMode():
    rospy.wait_for_service('mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='OFFBOARD')
    except rospy.ServiceException:
        print("service set_mode call failed. Offboard Mode could not be set.")


state = State()
def stateCb(msg):
    state.armed = msg.armed

local_pos = PoseStamped()
def posCb(msg):
    local_pos.pose.position.x = msg.pose.position.x
    local_pos.pose.position.y = msg.pose.position.y
    local_pos.pose.position.z = msg.pose.position.z

pos_hold = PositionTarget()
# set the flag to use position setpoints and yaw angle
pos_hold.type_mask = int('010111111000', 2)
# LOCAL_NED
pos_hold.coordinate_frame = 1
pos_hold.position.x = 0
pos_hold.position.y = 0
pos_hold.position.z = 5

fwd_vel = Twist()
fwd_vel.linear.x = 0.75
fwd_vel.linear.y = 0
fwd_vel.linear.z = 0

left_vel = Twist()
left_vel.linear.x = 0
left_vel.linear.y = -0.75
left_vel.linear.z = 0

right_vel = Twist()
right_vel.linear.x = 0
right_vel.linear.y = 0.75
right_vel.linear.z = 0

cw_yaw = Twist()
cw_yaw.angular.x = 0
cw_yaw.angular.y = 0
cw_yaw.angular.z = -0.75

ccw_yaw = Twist()
ccw_yaw.angular.x = 0
ccw_yaw.angular.y = 0
ccw_yaw.angular.z = 0.75


def main_func():
    counter = 0
    rospy.init_node('line_follower_node', anonymous=True)
    vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
    sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    #starting video stream
    cap = cv2.VideoCapture('line_latest1.mp4')

    rate = rospy.Rate(20) # 10hz
    rospy.Subscriber('mavros/state', State, stateCb)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, posCb)

    while not state.armed:
        setArm()
        rate.sleep()

    setOffboardMode()

    # Check if camera opened successfully
    if (cap.isOpened()== False):
        print("Error opening video stream or file")

    alt_sp = np.array((0, 0, 5))
    alt_offset = 0.3
    checkpoint1 = False
    ang_offset = 1.5
    x_offset = 42
    #Twist control_msg
    # Read until video is completed
    while cap.isOpened() and (not rospy.is_shutdown()):
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
        # Capture frame-by-frame
        pos = np.array((local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z))

        if np.linalg.norm(alt_sp - pos) < alt_offset:
            checkpoint1 = True
        if checkpoint1 == False:
            sp_pub.publish(pos_hold)
        if checkpoint1 == True:
            ret, frame = cap.read()
            #print(frame.shape)
            frame = cv2.resize(frame, (640, 360))
            # Convert the img to grayscale
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            #kernel = 9
            #gray_blur = cv2.GaussianBlur(gray,(kernel, kernel),0)
            kernel2 = np.ones((17,17),np.uint8)
            erosion = cv2.dilate(gray,kernel2,iterations = 3)
            # Apply edge detection method on the image
            edges = cv2.Canny(erosion,thresh_min,thresh_max,apertureSize = 3)
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
                    center_x = (x1+x2)/2
                    center_y = (y1+y2)/2
                    error_x = center_x - 320
                    if (x2 - x1 == 0):
                        ang = 0
                    else:
                        ang = -1*(math.atan((y2 - y1)/(x2 -x1)))
                # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
                # (0,0,255) denotes the colour of the line to be
                #drawn. In this case, it is red.
                    ang = round(ang, 2)
                    cv2.line(frame,(x1,y1), (x2,y2), (0,0,255),5)
                    cv2.line(frame,(320,0), (320,360), (0,255,255), 3)
                    cv2.putText(frame, str(ang), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, str(error_x), (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Display the resulting frame
            cv2.imshow('Frame',frame)
            print("frame processed")

            if abs(ang) > ang_offset:
                if theta > 0:
                    vel_pub.publish(cw_yaw)
                    print("Clockwise...  ")
                if ang < 0:
                    vel_pub.publish(ccw_yaw)
                    print("Anti Clockwise...  ")
            if abs(ang) < ang_offset:
                if abs(error_x) > x_offset:
                    if error_x > 0:
                        vel_pub.publish(left_vel)
                        print("Positive...  ")
                    if error_x < 0:
                        vel_pub.publish(right_vel)
                        print("Negative...  ")
            if ((abs(ang) < ang_offset) and (abs(error_x) < x_offset)):
                vel_pub.publish(fwd_vel)
                print("Forward...  ")

            # Press Q on keyboard to  exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                #cv2.imwrite("defective frame.jpg", frame)
                break
            counter += 1

        rate.sleep()
        time.sleep(0.3)

    # When everything done, release the video capture object
    cap.release()

    # Closes all the frames
    cv2.destroyAllWindows()


#main_func()
if __name__ == '__main__':
    try:
        main_func()
    except rospy.ROSInterruptException:
        pass
