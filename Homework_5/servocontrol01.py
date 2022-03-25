from itertools import cycle
import RPi.GPIO as gpio
import time
import cv2

def Open(pwm):
    pwm.ChangeDutyCycle(11.5)
    time.sleep(1.5)

def Half(pwm):
    pwm.ChangeDutyCycle(9.5)
    time.sleep(1.5)

def Close(pwm):
    pwm.ChangeDutyCycle(6.75)
    time.sleep(1.5)

def SetCycle(pwm):
    while True:

        Open(pwm)       
        Half(pwm)
        Close(pwm)
        Half(pwm)

def Timelapse(cap, duty_cycle):

    s, img = cap.read()
    img = cv2.flip(img, 0)
    if s:    # frame captured without any errors
        cv2.imshow("cam-test", img)
        cv2.putText(img, f'Dutycycle: {duty_cycle}',  (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.destroyWindow("cam-test")
        # output the frame
        out.write(img)

if __name__ == "__main__":

    cap = cv2.VideoCapture(0)

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('servo_control_01.mp4', fourcc, 20.0, (640, 480))

    gpio.setmode(gpio.BOARD)
    servo_pin = 36
    frequency = 50
    gpio.setup(servo_pin, gpio.OUT)
    pwm = gpio.PWM(servo_pin, frequency)
    duty_cycle = 11.5
    pwm.start(duty_cycle)
    cycle_down = True

    while(True):
        Timelapse(cap, duty_cycle)
        if cycle_down == True:
            if duty_cycle < 6.75:
                cycle_down = False
                time.sleep(0.5)
                
            else:
                duty_cycle = duty_cycle - 0.1
                pwm.ChangeDutyCycle(duty_cycle)
                time.sleep(0.5)
                
        elif cycle_down == False:
            if duty_cycle > 11.5:
                break

            else:
                duty_cycle = duty_cycle + 0.1
                pwm.ChangeDutyCycle(duty_cycle)
                time.sleep(0.5)

    # Close the window / Release webcam
    cap.release()
    
    # After we release our webcam, we also release the output
    out.release() 

    # De-allocate any associated memory usage 
    cv2.destroyAllWindows()
    
    pwm.stop()
    gpio.cleanup()
