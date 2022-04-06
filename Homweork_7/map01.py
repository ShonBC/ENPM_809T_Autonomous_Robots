
import RPi.GPIO as gpio
import time
import numpy as np

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

        # PID terms
        self.dt = 0.1 # Time step
        self.prev_err = 0
        self.integral = 0
        self.min_val = 0
        self.max_val = 2

        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.0

        # Robot properties
        self.gear_ratio = 120 / 1 # 1:120
        self.wheel_diameter = 0.065 # Meters
        self.tics_per_rev = 20 # Number of encoder tics per 1 wheel revolution
        self.drive_constant = self.tics_per_rev / (2 * np.pi * (self.wheel_diameter / 2))
        # self.gear_ratio * (1 / (2 * np.pi * (self.wheel_diameter / 2))) # Number of motor revolutions to drive 1 meter

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

    def Forward(self, distance):
        """Move robot forward for 'distance' meters

        Args:
            distance (int): Distance in meters for robot to drive forward
        """
 
        encoder_tics = self.drive_constant * distance # Total encoder tics to drive the desired distance
        print(encoder_tics)
        # Left wheel
        gpio.output(self.lb_motor_pin, True)
        gpio.output(self.lf_motor_pin, False)
        self.lpwm.start(self.motor_dut_cycle)
                
        # Right wheel
        gpio.output(self.rb_motor_pin, False)
        gpio.output(self.rf_motor_pin, True)
        self.rpwm.start(self.motor_dut_cycle)
        
        counterBR = np.uint64(0)
        counterFL = np.uint64(0)

        buttonBR = int(0)
        buttonFL = int(0)

        i = 0
        while i <= encoder_tics:

            stateBR = gpio.input(self.right_encoder_pin)
            stateFL = gpio.input(self.left_encoder_pin)

            # print(f'counterBR = {counterBR}, counterFL = {counterFL}, GPIO BRstate: {stateBR}, GPIO FLstate: {stateFL}')
            
            if int(stateBR) != int(buttonBR):
                buttonBR = int(stateBR)
                counterBR += 1
                

            if int(stateFL) != int(buttonFL):
                buttonFL = int(stateFL)
                counterFL += 1
                
            
            if counterBR >= encoder_tics:
                self.rpwm.stop()

            if counterFL >= encoder_tics:
                self.lpwm.stop()

            # PID tunning
            if counterBR > counterFL: 
                
                scale = self.PID(counterBR, counterFL)
                # print(f'Left Scale: {self.motor_dut_cycle * scale}')
                speed_update = min(self.motor_dut_cycle * scale, 100)
                self.rpwm.ChangeDutyCycle(speed_update)
                print(f'Left: {counterFL} Speed: {speed_update} Right: {counterBR}')

            if counterFL > counterBR:

                scale = self.PID(counterFL, counterBR)
                # print(f'Right Scale: {self.motor_dut_cycle * scale}')
                speed_update = min(self.motor_dut_cycle * scale, 100)
                self.lpwm.ChangeDutyCycle(speed_update)
                # print(f'Left: {counterFL} Right: {counterBR} Speed: {speed_update}')

            
            # if counterFL == counterBR:

            #     self.rpwm.ChangeDutyCycle(self.motor_dut_cycle)
            #     self.lpwm.ChangeDutyCycle(self.motor_dut_cycle)

            # Break when both encoder counts reached the desired total
            if counterBR >= encoder_tics and counterFL >= encoder_tics:
                break

        # # Wait
        # time.sleep(tf)

    def Reverse(self, distance):
        """Move robot in reverse for 'distance' meters

        Args:
            distance (int): Distance in meters for robot to drive in reverse
        """
 
        encoder_tics = self.drive_constant * self.tics_per_rev * distance # Total encoder tics to drive the desired distance

        # Left wheel
        gpio.output(self.lb_motor_pin, False)
        gpio.output(self.lf_motor_pin, True)
        self.lpwm.start(self.motor_dut_cycle)
        
        # Right wheel
        gpio.output(self.rb_motor_pin, True)
        gpio.output(self.rf_motor_pin, False)
        self.rpwm.start(self.motor_dut_cycle)

        # Wait
        time.sleep(distance)

    def PID(self, target, present):
        """A function which computes the PID controller output value. 'target' is used to store the setpoint
        and 'present' is used to store the current value
        Steps to calculate output :
        1) err is the difference between the target and the present value
        2) The proportional term is Kp times the error
        3) The error is multiplied with the time step dt and added to the integral variable
        4) The integral term is Ki times the integral variable
        5) The derivate term is Kd times the difference in present error and previous error divided by the time step
        6) Total output is the bounded (withing min and max) sum of the proportional, integral, and derivate term 

        Args:
            target (float): Desired final velocity
            present (float): Current velocity

        Returns:
            float: Final value calculated by PID controller
        """
        
        err = target - present

        p_term = self.kp * err

        self.integral = self.integral + (err * self.dt)

        i_term = self.ki * self.integral

        d_term = self.kd * (err - self.prev_err) / self.dt

        output = p_term + i_term + d_term

        self.prev_err = err

        output = max(self.min_val, min(self.max_val, output))

        return output

if __name__ == '__main__':

    robot = Robot()
    # robot.OpenGripper()
    # robot.CloseGripper()
    robot.Forward(int(input()))
    # robot.Reverse(4)
    robot.GameOver()
