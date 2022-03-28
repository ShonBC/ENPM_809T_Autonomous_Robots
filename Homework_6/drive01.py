'''Teleop Baron Bot for navigation and gripper control with collision avoidance.
'''

import RPi.GPIO as gpio
import time

def Init():
    """Define motor pins
    """

    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT) # IN1
    gpio.setup(33, gpio.OUT) # IN2
    gpio.setup(35, gpio.OUT) # IN3
    gpio.setup(37, gpio.OUT) # IN4

    gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP) # Setup encoder attached to pin 7
    gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP) # Setup encoder attached to pin 12

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

def main():

    # Assign pins
    lw