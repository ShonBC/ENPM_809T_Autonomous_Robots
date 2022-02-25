# Track green arrow and determine its orientation

import cv2
import numpy as np
import time   

def ColorRange(color= 'green'):

    if color == 'red':
        low_red = np.array([0, 34, 181])
        high_red = np.array([179, 140, 255])
        return low_red, high_red

    elif color == 'green':
        # Pi
        low_green = np.array([80, 116, 78])
        high_green = np.array([94, 185, 169])
        # low_green = np.array([30, 70, 220])
        # high_green = np.array([83, 255, 255])

        # Host
        # low_green = np.array([50, 100, 70])
        # high_green = np.array([90, 255, 255])
        return low_green, high_green

    elif color == 'yellow':
        low_yellow = np.array([0, 134, 134])
        high_yellow = np.array([179, 255, 255])
        return low_yellow, high_yellow

def Orientation(frame, mask):
    """Detect Arrow and determine its orientation. 

    Args:
        frame (Mat): Frame to be processed
        mask (Mat): Frame prefiltered to show a specific color

    Returns:
        frame (Mat): Frame being processed with arrow orientation written over it
        orientation (string): Orientation of the detected arrow
    """

    gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

    orientation = ''

    # Shi-Tomasi Corner Detection parameters
    feature_params = dict(maxCorners=7,
                            qualityLevel=0.01,
                            minDistance=30,
                            blockSize=10)
    
    corners = cv2.goodFeaturesToTrack(gray, mask=None, **feature_params)
    
    if corners is not None and len(corners) > 5:
        # Fit an ellipse to the corners detected
        ellipse_center, (MA, ma), angle = cv2.fitEllipse(corners)
        cv2.circle(frame, (int(ellipse_center[0]), int(ellipse_center[1])), 2, (255, 0, 0), -1)

        # Check if the object detected fits an ellipse (arrow) by a threshold
        if ma / MA > 1.25:
        
            # Find the momentum to detect the arrow head orientation
            moments = cv2.moments(gray, True)
            moment_center = (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))

            cv2.circle(frame, (int(moment_center[0]), int(moment_center[1])), 2, (0, 0, 255), -1)

            moments = cv2.getRotationMatrix2D(moment_center, angle-90, 1)
            moments = cv2.moments(gray, True)
            moment_center = (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))
            corners = np.asarray(corners)
            corners = np.squeeze(corners, axis = 1)
            x_max = np.argmax(corners[:, 0])
            y_max = np.argmax(corners[:, 1])

            if 45 < angle < 135:
                if corners[y_max, 0] > moment_center[0]:
                    orientation = 'right'
                else:
                    orientation = 'left'
            else:
                if corners[y_max, 0] > moment_center[0]:
                    orientation = 'down'
                else:
                    orientation = 'up'

            cv2.putText(frame, f'{orientation}', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            # cv2.imshow('Filtered Frame', blur_img)
    return frame, orientation

def main():

    # This will return video from the first webcam on your computer.
    cap = cv2.VideoCapture(0)
    f = open('hw4data.txt','a')
    
    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('arrow_detection.mp4', fourcc, 20.0, (640, 480))
    
    # loop runs if capturing has been initialized. 
    while(True):
        start = time.time()
        # Reads frames from a camera 
        # ret checks return at each frame
        ret, frame = cap.read() 

        if not ret:
            break
        
        # Blur and mask frame for specified color
        k = 5
        blur = cv2.GaussianBlur(frame, (k, k), cv2.BORDER_DEFAULT)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        low_thresh, high_thresh = ColorRange()
        mask = cv2.inRange(hsv, low_thresh, high_thresh)
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Perform Closing (morphology) to filter noise
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.dilate(mask, kernel)
        mask = cv2.erode(mask, kernel)
        
        # Detect and determine orientation of arrow
        frame, orientation = Orientation(frame, mask)
        
        # The original input frame is shown in the window 
        cv2.imshow('Original', frame)
        
        stop = time.time()
        time_delta = stop - start
        outstring = str(time_delta) + '\n'
        # print(outstring)
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

if __name__ == '__main__':

    main()
