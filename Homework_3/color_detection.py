# Track colors and draw bounding boxes

import cv2
import numpy as np

def ColorRange(color: str):

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

    color = input('Enter color to track. ')
        
    # This will return video from the first webcam on your computer.
    cap = cv2.VideoCapture('Homework_3/stoplight.mp4')  
    
    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('color_detection.mp4', fourcc, 20.0, (640, 480))
    
    # loop runs if capturing has been initialized. 
    while(True):
        # reads frames from a camera 
        # ret checks return at each frame
        ret, frame = cap.read() 

        if not ret:
            break
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_thresh, high_thresh = ColorRange(color)
        mask = cv2.inRange(hsv, low_thresh, high_thresh)
        color_mask = cv2.bitwise_and(frame, frame, mask=mask)

        
        # output the frame
        out.write(frame) 
        
        # The original input frame is shown in the window 
        cv2.imshow('Original', frame)
        cv2.imshow(f'{color} Mask', mask)  
        compare_frame = np.hstack((frame, hsv, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
        cv2.imshow('hstack: ', compare_frame)
        
        # Wait for 'a' key to stop the program 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Close the window / Release webcam
    cap.release()
    
    # After we release our webcam, we also release the output
    out.release() 
    
    # De-allocate any associated memory usage 
    cv2.destroyAllWindows()
