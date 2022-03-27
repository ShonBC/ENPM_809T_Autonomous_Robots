'''Teleop Baron Bot for navigation and gripper control.
'''

import RPi.GPIO as gpio
import time
import cv2

def Init():
    """Assign RPI pins
    """

    gpio.setmode(gpio.BOARD)

    # Motor pins
    gpio.setup(31, gpio.OUT) # IN1
    gpio.setup(33, gpio.OUT) # IN2
    gpio.setup(35, gpio.OUT) # IN3
    gpio.setup(37, gpio.OUT) # IN4

    # Servo pins
    servo_pin = 36
    frequency = 50
    gpio.setup(servo_pin, gpio.OUT)
    pwm = gpio.PWM(servo_pin, frequency)
    duty_cycle = 11.5
    pwm.start(duty_cycle)

    # Distance sensor pins
    trig = 16
    echo = 18
    gpio.setup(trig, gpio.OUT)
    gpio.setup(echo, gpio.IN)
    gpio.output(trig, False)

    return pwm

def GameOver():
    """Set all pins to low
    """

    # Motor pins
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)

    # Servo pins
    gpio.output(36, False)

    # Distance sensor pins
    gpio.output(16, False)

    gpio.cleanup()

def Distance():
    """Generate pulse signal to measure distance of objects in front of Baron Bot

    Returns:
        float: Distance of objects detected by distance sensor in [cm]
    """

    Init()

    trig = 16
    echo = 18

    # Ensure output has no value
    gpio.output(trig, False)
    time.sleep(0.01)

    # Generate trigger pulse
    gpio.output(trig, True)
    time.sleep(0.00001)
    gpio.output(trig, False)

    # Generate echo time signal
    while gpio.input(echo) == 0:
        pulse_start = time.time()

    while gpio.input(echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    # Convert time to distance in [cm] using speed of sound
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    # Cleanup gpio pins & return distance estimate
    gpio.cleanup()

    print(f'Distance: {distance}')
    return distance

def Forward(tf):
    """Move robot forward for 'tf' seconds

    Args:
        tf (int): Time in seconds for robot to drive forward
    """

    Init()
    # Left wheel
    gpio.output(31, True)
    gpio.output(33, False)

    # Right wheel
    gpio.output(35, False)
    gpio.output(37, True)

    # Wait
    time.sleep(tf)

    # Set all pins to low and cleanup
    GameOver()
    gpio.cleanup()

def Backward(tf):
    """Move robot backward for 'tf' seconds

    Args:
        tf (int): Time in seconds for robot to drive forward
    """

    Init()
    # Left wheel
    gpio.output(31, False)
    gpio.output(33, True)

    # Right wheel
    gpio.output(35, True)
    gpio.output(37, False)

    # Wait
    time.sleep(tf)

    # Set all pins to low and cleanup
    GameOver()
    gpio.cleanup()

def RightPiv(tf):
    """Pivot robot to the right for 'tf' seconds

    Args:
        tf (int): Time in seconds for robot to drive forward
    """

    Init()
    # Left wheel
    gpio.output(31, True)
    gpio.output(33, False)

    # Right wheel
    gpio.output(35, True)
    gpio.output(37, False)

    # Wait
    time.sleep(tf)

    # Set all pins to low and cleanup
    GameOver()
    gpio.cleanup()

def LeftPiv(tf):
    """Pivot robot to the left for 'tf' seconds

    Args:
        tf (int): Time in seconds for robot to drive forward
    """

    Init()
    # Left wheel
    gpio.output(31, False)
    gpio.output(33, True)

    # Right wheel
    gpio.output(35, False)
    gpio.output(37, True)

    # Wait
    time.sleep(tf)

    # Set all pins to low and cleanup
    GameOver()
    gpio.cleanup()

def OpenGripper():
    """Fully open gripper
    """

    pwm = Init()

    pwm.ChangeDutyCycle(11.5)
    time.sleep(1.5)
    GameOver()

def CloseGripper():
    """Fully close gripper
    """
    
    pwm = Init()

    pwm.ChangeDutyCycle(6.75)
    time.sleep(1.5)
    GameOver()

def KeyInput(event):
    """Operate robot through user input to drive and open/close gripper

    Args:
        event (str): 'w', 's', 'a', 'd', 'o', 'c', 'p' to choose an action for the robot 
    """

    Init()

    print(f'Key: {event}')

    key_press = event
    tf = 0.5

    # Measure distance of objects before following a command
    Distance()

    if key_press.lower() == 'w':
        Forward(tf)
    elif key_press == 's':
        Backward(tf)
    elif key_press == 'a':
        LeftPiv(tf)
    elif key_press == 'd':
        RightPiv(tf)
    elif key_press == 'o':
        OpenGripper()
    elif key_press == 'c':
        CloseGripper()
    else:
        print('Invlaid key pressed!!')

if __name__ == '__main__':

    while True:

        key_press = input("Select Action: 'w' - Forward \n \
              's' - Backward \n \
              'a' - Pivot Left \n \
              'd' - Pivot Right \n \
              'o' - Open Gripper \n \
              'c' - Close Gripper \n \
              'p' - Exit Program \n")

        if key_press.lower() == 'p':
            break
        
        KeyInput(key_press)
