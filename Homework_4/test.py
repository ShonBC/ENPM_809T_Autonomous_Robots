# Track colors and draw bounding boxes

import cv2
import imutils
import numpy as np
import time

def ColorRange(color= 'green'):

    if color == 'red':
        low_red = np.array([0, 34, 181])
        high_red = np.array([179, 140, 255])
        return low_red, high_red

    elif color == 'green':
        low_green = np.array([50, 100, 70])
        high_green = np.array([90, 255, 255])
        return low_green, high_green

    elif color == 'yellow':
        low_yellow = np.array([0, 134, 134])
        high_yellow = np.array([179, 255, 255])
        return low_yellow, high_yellow

if __name__ == '__main__':

    # color = input('Enter color to track. ')

    # This will return video from the first webcam on your computer.
    cap = cv2.VideoCapture('cal_.jpg')  
    # f = open('hw3data.txt','a')
    
    # # Define the codec and create VideoWriter object
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # out = cv2.VideoWriter('color_detection.mp4', fourcc, 20.0, (640, 480))
    
    # loop runs if capturing has been initialized. 
    frame = cv2.imread('cal_.jpg')
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low_thresh, high_thresh = ColorRange()
    mask = cv2.inRange(hsv, low_thresh, high_thresh)
    # color_mask = cv2.bitwise_and(frame, frame, mask=mask)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)



    k = 5
    blur = cv2.GaussianBlur(mask, (k, k), cv2.BORDER_DEFAULT)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

    # Detect corners
    max_corners = 7
    min_distance = 30
    corners = cv2.goodFeaturesToTrack(gray, max_corners, 0.5, min_distance)
    if corners is None:
        pass
    else:
        corners = np.int0(corners)

    # cmd = Orientation(corners)
        if len(corners) > 0:
            for i in corners:
                x, y = i.ravel()
                cv2.circle(frame, (x, y), 3, 255, -1)

    
    # The original input frame is shown in the window 
    cv2.imshow('Original', frame)
    cv2.imshow('Green Mask', mask)  
    compare_frame = np.hstack((frame, hsv, blur))
    cv2.imshow('hstack: ', compare_frame)
    cv2.waitKey(0)
    # stop = time.time()
    # time_delta = stop - start
    # outstring = str(time_delta) + '\n'
    # print(outstring)
    # f.write(outstring)
    # Wait for 'a' key to stop the program 
   
    
    
    # After we release our webcam, we also release the output
    # out.release() 
    
    # De-allocate any associated memory usage 
    cv2.destroyAllWindows()
