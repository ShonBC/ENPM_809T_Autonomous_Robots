import RPi.GPIO as gpio
import time

def Open(pwm):
    pwm.ChangeDutyCycle(11.5)
    time.sleep(1.5)

def Half(pwm):
    pwm.ChangeDutyCycle(9.5)
    time.sleep(1.5)

def Close(pwm):
    pwm.ChangeDutyCycle(6.75)
    time.sleep(1.5)

if __name__ == "__main__":

    gpio.setmode(gpio.BOARD)
    servo_pin = 36
    frequency = 50
    gpio.setup(servo_pin, gpio.OUT)
    pwm = gpio.PWM(servo_pin, frequency)
    pwm.start(6.75)
    
    while True:

        Open(pwm)       
        Half(pwm)
        Close(pwm)
        Half(pwm)


    pwm.stop()
    gpio.cleanup()
