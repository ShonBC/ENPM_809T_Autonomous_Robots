
import cv2 as cv
import os
import RPi.GPIO as gpio

# Set up gpio pins
gpio.setmode(gpio.BOARD)
gpio.setup(36, gpio.OUT)

# Initialize pwm signal & move gripper to center
servo_pin = 36
frequency = 50
gpio.setup(servo_pin, gpio.OUT)
pwm = gpio.PWM(servo_pin, frequency)
pwm.start(9.5) # Half open

# Initialize the video feed
cmd = 'sudo modprobe bcm2835-v4l2'
os.system(cmd)

# Open video capture
cap = cv.VideoCapture(0)

# Define detector
detector = cv.QRCodeDetector()

while True:

    ret, frame = cap.read()

    data, bbox, _ = detector.detectAndDecode(frame)

    if(bbox is not None):
        for i in range(len(bbox)):
            cv.line(frame, tuple(bbox[i][0]), tuple(bbox[(i + 1) % len(bbox)][0]), color = (0, 0, 255), thickness = 4)
            cv.putText(frame, data, (int(bbox[0][0][0]), int(bbox[0][0][1] - 10)), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    if data:
        print(f'Data: {data}')

        if data == 'HALF':
            pwm.ChangeDutyCycle(9.5)
        elif data == 'OPEN':
            pwm.ChangeDutyCycle(11.5)
        elif data == "CLOSE":
            pwm.ChangeDutyCycle(6.75)
    
    # Show results to screen
    cv.imshow('QR Code Decoder', frame)

    if cv.waitKey(1) == ord('q'):
        pwm.stop()
        gpio.cleanup()
        break

cap.release()
cv.destroyAllWindows()
