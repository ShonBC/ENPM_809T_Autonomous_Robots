"""Test the encoders on Baron Bot. Once running, manually spin the 
wheel plugged into pin 12 on the RPI to track the encoder counts.
"""

import RPi.GPIO as gpio
import numpy as np


def init():
    """Initialize GPIO pins
    """
    gpio.setmode(gpio.BOARD)

    gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)

def gameover():
    gpio.cleanup()

if __name__ == '__main__':
    init()

    counter = np.uint64(0)

    button = int(0)

    while True:

        if int(gpio.input(12) != int(button)):
            button = int(gpio.input(12))
            counter += 1
            print(counter)

        if counter >= 20:
            gameover()
            print('Thanks for playing')
            break
