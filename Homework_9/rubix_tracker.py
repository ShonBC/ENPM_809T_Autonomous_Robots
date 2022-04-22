import baron_bot
import cv2
import imutils
import RPi.GPIO as gpio


def TrackColor(robot):
    cap = cv2.VideoCapture(0)

    ret, frame = cap.read()
    if ret:    # frame captured without any errors

        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_thresh, high_thresh = robot.ColorRange()
        mask = cv2.inRange(hsv, low_thresh, high_thresh)

        # Find the contours
        cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # Draw contours and track center
        for c in cnts:

            # (x, y), radius = cv2.minEnclosingCircle(c)
            # center = (int(x), int(y))
            # radius = int(radius)

            rect = cv2.boundingRect(c)
            min_box_size = 15
            if rect[2] < min_box_size or rect[3] < min_box_size:
                continue

            x, y, w, h = rect

            center = (int(x + w / 2), int(y + h / 2))

            cv2.rectangle(frame, (x, y), (x + w, y + h),
                          color=(0, 255, 255), thickness=2)

            robot.DistFromCenter(x)
            distance = robot.ImgDistance(w)

            # cv2.circle(frame, center, radius,
            #            color=(0, 255, 255), thickness=2)
            cv2.circle(frame, center, 1, color=(0, 0, 255), thickness=4)

        # The original input frame is shown in the window
        cv2.imshow('Original', frame)
        cv2.waitKey(0)
        cv2.imwrite("cal_.jpg", frame)  # Save image

    # Close the window / Release webcam
    cap.release()

    # De-allocate any associated memory usage
    cv2.destroyAllWindows()


if __name__ == '__main__':
    robot = baron_bot.Robot()

    for i in range(10):
        TrackColor(robot)
    robot.GameOver()
    gpio.cleanup()
