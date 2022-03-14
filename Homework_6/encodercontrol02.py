"""Spin wheel connected to pin 12 on RPI for 1 revolution. Save the 
encoder states to a txt file for processing and analysis. 
"""

import RPi.GPIO as gpio
import numpy as np
import time

def Init():
    """Define motor pins
    """

    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT) # IN1
    gpio.setup(33, gpio.OUT) # IN2
    gpio.setup(35, gpio.OUT) # IN3
    gpio.setup(37, gpio.OUT) # IN4

    gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP) # Setup encoder attached to pin 12

def GameOver():
    """Set all pins to low
    """

    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)

    gpio.cleanup()

def main():
    Init()

    # Save encoder states to txt file
    f = open('encoderstates.txt','a')

    counter = np.uint64(0)

    button = int(0)

    servo_pin = 37
    frequency = 50

    pwm = gpio.PWM(servo_pin, frequency)

    val = 14
    pwm.start(val)
    time.sleep(0.1)

    for i in range(0, 200000):

        state = gpio.input(12)

        print(f'counter = {counter}, GPIO state: {state}')
        outstring = str(state) + '\n'
        f.write(outstring)

        if int(gpio.input(12)) != int(button):
            button = int(gpio.input(12))
            counter += 1

        if counter >= 20:
            pwm.stop()
            GameOver()
            print('Thanks for playing')
            break

if __name__ == '__main__':
    main()
