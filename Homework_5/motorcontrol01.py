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

def GameOver():
    """Set all pins to low
    """

    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)

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

def LeftPiv(tf):
    """Pivot robot to the left for 'tf' seconds

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

print('start')
Forward(2)
Backward(2)
RightPiv(2)
LeftPiv(2)
print('end')
