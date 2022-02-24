import cv2 as cv

# initialize the camera
cam = cv.VideoCapture(0)   # 0 -> index of camera
s, img = cam.read()
if s:    # frame captured without any errors
    cv.imshow("cam-test",img)
    cv.waitKey(0)
    cv.destroyWindow("cam-test")
    cv.imwrite("cal_.jpg",img) #save image
