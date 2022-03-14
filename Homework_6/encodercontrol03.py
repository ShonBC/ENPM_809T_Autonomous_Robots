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

    gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP) # Setup encoder attached to pin 7
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
    br = open('BRencoderstates.txt','a')
    fl = open('FLencoderstates.txt','a')

    counterBR = np.uint64(0)
    counterFL = np.uint64(0)

    buttonBR = int(0)
    buttonFL = int(0)

    servo_pin = 37
    frequency = 50

    pwm = gpio.PWM(servo_pin, frequency)

    val = 16
    pwm.start(val)
    time.sleep(0.1)

    for i in range(0, 200000):

        stateBR = gpio.input(12)
        stateFL = gpio.input(7)

        print(f'counterBR = {counterBR}, counterFL = {counterFL}, GPIO BRstate: {stateBR}, GPIO FLstate: {stateFL}')
        
        # Save encoder states to txt files
        broutstring = str(stateBR) + '\n'
        floutstring = str(stateFL) + '\n'
        br.write(broutstring)
        fl.write(floutstring)

        if int(stateBR) != int(buttonBR):
            buttonBR = int(stateBR)
            counterBR += 1

        if int(stateFL) != int(buttonFL):
            buttonFL = int(stateFL)
            counterFL += 1

        if counterBR >= 20:
            pwm.stop()
            GameOver()
            print('Thanks for playing')
            break
