
import RPi.GPIO as gpio
import time

class Robot:
    
    def __init__(self):
        # Motor pins
        self.motor_frequency = 50
        self.motor_dut_cycle = 60 # Controls speed
        self.lb_motor_pin = 31
        self.lf_motor_pin = 33
        self.rb_motor_pin = 35
        self.rf_motor_pin = 37

        # Encoder pins
        self.left_encoder_pin = 7
        self.right_encoder_pin = 12
        
        # Gripper pins
        self.servo_frequency = 50
        self.open_servo_duty_cycle = 11.5
        self.close_servo_duty_cycle = 6.75
        self.servo_pin = 36

        # Distance sensor pins
        self.trig = 16
        self.echo = 18

        # PWM signals
        self.lpwm = 0
        self.rpwm = 0
        self.gpwm = 0

        self.InitGpio()

    def InitGpio(self):
        """Assign RPI pins and initialize pwm signals
        """

        gpio.setmode(gpio.BOARD)

        # Left Motor pins
        gpio.setup(self.lb_motor_pin, gpio.OUT) # IN1
        gpio.setup(self.lf_motor_pin, gpio.OUT) # IN2
        self.lpwm = gpio.PWM(self.lb_motor_pin, self.motor_frequency) # Control both left wheels

        # Right Motor pins
        gpio.setup(self.rb_motor_pin, gpio.OUT) # IN3
        gpio.setup(self.rf_motor_pin, gpio.OUT) # IN4
        self.rpwm = gpio.PWM(self.rf_motor_pin, self.motor_frequency) # Control both right wheels

        # Encoder pins
        gpio.setup(self.left_encoder_pin, gpio.IN, pull_up_down = gpio.PUD_UP) # (Left side) Setup encoder attached to pin 7
        gpio.setup(self.right_encoder_pin, gpio.IN, pull_up_down = gpio.PUD_UP) # (Right side) Setup encoder attached to pin 12 

        # Servo pins
        gpio.setup(self.servo_pin, gpio.OUT)
        self.gpwm = gpio.PWM(self.servo_pin, self.servo_frequency)
        self.gpwm.start(self.open_servo_duty_cycle)

        # Distance sensor pins
        gpio.setup(self.trig, gpio.OUT)
        gpio.setup(self.echo, gpio.IN)
        gpio.output(self.trig, False)

    def GameOver(self):
        """Set all pins to low
        """

        # Motor pins
        gpio.output(self.lb_motor_pin, False)
        gpio.output(self.lf_motor_pin, False)
        gpio.output(self.rb_motor_pin, False)
        gpio.output(self.rf_motor_pin, False)

        # Servo pins
        gpio.output(self.servo_pin, False)
        
        # Distance sensor pins
        gpio.output(self.trig, False)

        gpio.cleanup()

    def CloseGripper(self):
        """Fully close gripper
        """

        self.gpwm.ChangeDutyCycle(self.close_servo_duty_cycle)
        time.sleep(1.5)
    
    def OpenGripper(self):
        """Fully open gripper
        """

        self.gpwm.ChangeDutyCycle(self.open_servo_duty_cycle)
        time.sleep(1.5)

    def Forward(self, tf):
        """Move robot forward for 'tf' seconds

        Args:
            tf (int): Time in seconds for robot to drive forward
        """

        # Left wheel
        self.lpwm.start(self.motor_dut_cycle)
        
        # Right wheel
        self.rpwm.start(self.motor_dut_cycle)

        # Wait
        time.sleep(tf)
    
    def Backward(self, tf):
        """Move robot backward for 'tf' seconds

        Args:
            tf (int): Time in seconds for robot to drive forward
        """

        # Left wheel
        self.lpwm.start(50)
        
        # Right wheel
        self.rpwm.start(50)

        # Wait
        time.sleep(tf)

    def PID(self):
        left_speed = 0 
        right_speed = 0



if __name__ == '__main__':

    robot = Robot()
    robot.OpenGripper()
    # robot.CloseGripper()
    robot.Forward(8)
    # robot.Backward(4)
    robot.GameOver()
