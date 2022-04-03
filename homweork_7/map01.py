
import RPi.GPIO as gpio
import time

class Robot:
    
    def __init__(self):
        # Motor pins
        self.motor_frequency = 50
        self.lb_motor_pin = 31
        self.lf_motor_pin = 33
        self.rb_motor_pin = 35
        self.rf_motor_pin = 37

        # Gripper pins
        self.servo_frequency = 50
        self.servo_pin = 36

        # Distance sensor pins
        self.trig = 16
        self.echo = 18

    lpwm = gpio.PWM(31, self.motor_frequency) # Control both left wheels
    rpwm = gpio.PWM(37, self.motor_frequency) # Control both right wheels


def Init():
    """Assign RPI pins
    """

    gpio.setmode(gpio.BOARD)

    # Left Motor pins
    motor_frequency = 50
    gpio.setup(31, gpio.OUT) # IN1
    gpio.setup(33, gpio.OUT) # IN2

    lpwm = gpio.PWM(31, motor_frequency) # Control both left wheels

    # Right Motor pins
    gpio.setup(35, gpio.OUT) # IN3
    gpio.setup(37, gpio.OUT) # IN4

    rpwm = gpio.PWM(37, motor_frequency) # Control both right wheels

    # Servo pins
    servo_pin = 36
    servo_frequency = 50
    gpio.setup(servo_pin, gpio.OUT)
    gpwm = gpio.PWM(servo_pin, servo_frequency)
    duty_cycle = 11.5
    gpwm.start(duty_cycle)

    # Distance sensor pins
    trig = 16
    echo = 18
    gpio.setup(trig, gpio.OUT)
    gpio.setup(echo, gpio.IN)
    gpio.output(trig, False)

    return lpwm, rpwm, gpwm