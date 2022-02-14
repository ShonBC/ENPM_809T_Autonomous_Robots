# Track green and draw bounding circle. Can record result in .mp4 file.

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
    cap = cv2.VideoCapture(0)  
    f = open('hw3data.txt','a')
    
    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('color_detection.mp4', fourcc, 20.0, (640, 480))
    
    # loop runs if capturing has been initialized. 
    while(True):
        start = time.time()
        # reads frames from a camera 
        # ret checks return at each frame
        ret, frame = cap.read() 

        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_thresh, high_thresh = ColorRange()
        mask = cv2.inRange(hsv, low_thresh, high_thresh)

        # Perform Closing (morphology) to filter noise
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.dilate(mask, kernel)
        mask = cv2.erode(mask, kernel)
        color_mask = cv2.bitwise_and(frame, frame, mask=mask)

        # Find the contours 
        cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # Draw contours and track center
        for c in cnts:

            (x, y), radius = cv2.minEnclosingCircle(c)
            center = (int(x), int(y))
            radius = int(radius)

            cv2.circle(frame, center, radius, color=(0, 255, 255), thickness=2)
            cv2.circle(frame, center, 1, color=(0, 0, 255), thickness=4)

        # output the frame
        out.write(frame) 
        
        # The original input frame is shown in the window 
        # cv2.imshow('Original', frame)
        # cv2.imshow('Green Mask', mask)  
        compare_frame = np.hstack((frame, hsv, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
        cv2.imshow('Frame Comparison (BGR, HSV, Color Mask)', compare_frame)
        
        stop = time.time()
        time_delta = stop - start
        outstring = str(time_delta) + '\n'
        print(outstring)
        f.write(outstring)
        # Wait for 'a' key to stop the program 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Close the window / Release webcam
    cap.release()
    
    # After we release our webcam, we also release the output
    out.release() 
    
    # De-allocate any associated memory usage 
    cv2.destroyAllWindows()
