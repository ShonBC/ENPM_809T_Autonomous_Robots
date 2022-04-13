
import RPi.GPIO as gpio
import time
import numpy as np
import serial


class Robot:

    def __init__(self, monitor_encoders=False):

        # Save Encoder data to text files
        self.monitor_encoders = monitor_encoders

        # Motor pins
        self.motor_frequency = 50
        self.motor_dut_cycle = 45  # Controls speed
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

        # Identify serial connection on RPI for IMU
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)

        # PID terms
        self.dt = 0.1  # Time step
        self.prev_err = 0
        self.integral = 0
        self.min_val = 0
        self.max_val = 2

        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.0

        # Robot properties
        self.gear_ratio = 120 / 1  # 1:120
        self.wheel_diameter = 0.065  # Meters
        # self.wheel_base = 0.1524 # Meters (6")
        self.wheel_base = 0.1397  # Meters (5.5")
        # self.wheel_base = 0.127 # Meters (5")
        self.tics_per_rev = 20  # Number of encoder tics per 1 wheel revolution
        self.drive_constant = self.tics_per_rev \
            / (2 * np.pi * (self.wheel_diameter / 2))
        self.turning_perimeter = 2 * np.pi * (self.wheel_base)

    def InitGpio(self):
        """Assign RPI pins and initialize pwm signals
        """

        gpio.setmode(gpio.BOARD)

        # Left Motor pins
        gpio.setup(self.lb_motor_pin, gpio.OUT)  # IN1
        gpio.setup(self.lf_motor_pin, gpio.OUT)  # IN2

        # Control both left wheels
        self.lpwm = gpio.PWM(self.lb_motor_pin, self.motor_frequency)

        # Right Motor pins
        gpio.setup(self.rb_motor_pin, gpio.OUT)  # IN3
        gpio.setup(self.rf_motor_pin, gpio.OUT)  # IN4

        # Control both right wheels
        self.rpwm = gpio.PWM(self.rf_motor_pin, self.motor_frequency)

        # Encoder pins
        gpio.setup(self.left_encoder_pin, gpio.IN,
                   pull_up_down=gpio.PUD_UP)  # (Left) Setup pin 7 encoder
        gpio.setup(self.right_encoder_pin, gpio.IN,
                   pull_up_down=gpio.PUD_UP)  # (Right) Setup pin 12 encoder

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

    def MonitorEncoders(self, action, stateBR, stateFL):
        # Save encoder states to txt file
        br = open(f'{action}_BRencoderstates.txt', 'a')
        fl = open(f'{action}_FLencoderstates.txt', 'a')
        # Save encoder states to txt files
        broutstring = str(stateBR) + '\n'
        floutstring = str(stateFL) + '\n'
        br.write(broutstring)
        fl.write(floutstring)

    def Distance(self):
        """Generate pulse signal to measure distance of objects in front of
        Baron Bot

        Returns:
            float: Distance of objects detected by distance sensor in [cm]
        """

        # Ensure output has no value
        gpio.output(self.trig, False)
        time.sleep(0.01)

        # Generate trigger pulse
        gpio.output(self.trig, True)
        time.sleep(0.00001)
        gpio.output(self.trig, False)

        # Generate echo time signal
        while gpio.input(self.echo) == 0:
            pulse_start = time.time()

        while gpio.input(self.echo) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        # Convert time to distance in [cm] using speed of sound
        distance = pulse_duration * 17150
        distance = round(distance, 2)

        print(f'Distance Sensor Reading: {distance}')
        return distance

    def ReadIMU(self):
        """Read IMU data and return the value as a float

        Returns:
            float: x-axis orientation value
        """

        count = 0

        while count < 11:
            if(self.ser.in_waiting > 0):

                count += 1

                # Read serial stream
                line = self.ser.readline()

                # Avoid first n-lines of serial info
                if count > 10:

                    # Strip serial stream of extra characters
                    line = line.rstrip().lstrip()

                    line = str(line)
                    line = line.strip("'")
                    line = line.strip("b'")

                    # Return float
                    line = float(line)

                    return line

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

        # Total encoder tics to drive the desired distance
        encoder_tics = self.drive_constant * distance
        # print(f'Encoder Tics: {encoder_tics}')

        # Get Initial IMU angle reading
        init_angle = self.ReadIMU()
        print(f'angle: {init_angle}')

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

            # print(f'counterBR = {counterBR}, counterFL = {counterFL}, \
            # GPIO BRstate: {stateBR}, GPIO FLstate: {stateFL}')

            # Save encoder states to txt files
            if self.monitor_encoders is True:
                self.MonitorEncoders('Forward', stateBR, stateFL)

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
            if counterBR > counterFL:  # Double speed to match encoder counts

                speed_update = min(self.motor_dut_cycle * 2, 100)
                self.lpwm.ChangeDutyCycle(speed_update)
                # print(f'Left: {counterFL} Speed: {speed_update} \
                # Right: {counterBR}')

            if counterFL > counterBR:  # Double speed to match encoder counts

                speed_update = min(self.motor_dut_cycle * 2, 100)
                self.rpwm.ChangeDutyCycle(speed_update)
                # print(f'Left: {counterFL} Right: {counterBR} \
                # Speed: {speed_update}')

            if counterFL == counterBR:

                self.rpwm.ChangeDutyCycle(self.motor_dut_cycle)
                self.lpwm.ChangeDutyCycle(self.motor_dut_cycle)

            # Break when both encoder counts reached the desired total
            if counterBR >= encoder_tics and counterFL >= encoder_tics:
                break

    def Reverse(self, distance):
        """Move robot in reverse for 'distance' meters

        Args:
            distance (int): Distance in meters for robot to drive in reverse
        """

        # Total encoder tics to drive the desired distance
        encoder_tics = self.drive_constant * distance

        # Left wheel
        gpio.output(self.lb_motor_pin, False)
        gpio.output(self.lf_motor_pin, True)
        self.lpwm.start(self.motor_dut_cycle)

        # Right wheel
        gpio.output(self.rb_motor_pin, True)
        gpio.output(self.rf_motor_pin, False)
        self.rpwm.start(self.motor_dut_cycle)

        time.sleep(distance)

        counterBR = np.uint64(0)
        counterFL = np.uint64(0)

        buttonBR = int(0)
        buttonFL = int(0)

        i = 0
        while i <= encoder_tics:

            stateBR = gpio.input(self.right_encoder_pin)
            stateFL = gpio.input(self.left_encoder_pin)

            # print(f'counterBR = {counterBR}, counterFL = {counterFL}, \
            # GPIO BRstate: {stateBR}, GPIO FLstate: {stateFL}')

            # Save encoder states to txt files
            if self.monitor_encoders is True:
                self.MonitorEncoders('Reverse', stateBR, stateFL)

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
            if counterBR > counterFL:  # Double speed to match encoder counts

                speed_update = min(self.motor_dut_cycle * 2, 100)
                self.rpwm.ChangeDutyCycle(speed_update)
                # print(f'Left: {counterFL} Speed: {speed_update} \
                # Right: {counterBR}')

            if counterFL > counterBR:  # Double speed to match encoder counts

                speed_update = min(self.motor_dut_cycle * 2, 100)
                self.lpwm.ChangeDutyCycle(speed_update)
                # print(f'Left: {counterFL} Right: {counterBR} \
                # Speed: {speed_update}')

            if counterFL == counterBR:

                self.rpwm.ChangeDutyCycle(self.motor_dut_cycle)
                self.lpwm.ChangeDutyCycle(self.motor_dut_cycle)

            # Break when both encoder counts reached the desired total
            if counterBR >= encoder_tics and counterFL >= encoder_tics:
                break

    def LeftPiv(self, angle):
        """Pivot robot to the left for 'tf' seconds

        Args:
            angle (int): Angle in degrees for robot to turn left
        """

        fraction = angle / 360
        encoder_tics = self.drive_constant * self.turning_perimeter * fraction
        # print(self.drive_constant)
        # print(encoder_tics)

        # Left wheel
        gpio.output(self.lb_motor_pin, False)
        gpio.output(self.lf_motor_pin, True)
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

            # print(f'counterBR = {counterBR}, counterFL = {counterFL}, \
            # GPIO BRstate: {stateBR}, GPIO FLstate: {stateFL}')

            # Save encoder states to txt files
            if self.monitor_encoders is True:
                self.MonitorEncoders('LeftPiv', stateBR, stateFL)

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

            # Break when both encoder counts reached the desired total
            if counterFL >= encoder_tics:
                self.rpwm.stop()
                self.lpwm.stop()
                break

    def RightPiv(self, angle):
        """Pivot robot to the right for 'tf' seconds

        Args:
            angle (int): Angle in degrees for robot to turn right
        """

        fraction = angle / 360
        encoder_tics = self.drive_constant * self.turning_perimeter * fraction
        # print(self.drive_constant)
        # print(encoder_tics)

        # Left wheel
        gpio.output(self.lb_motor_pin, True)
        gpio.output(self.lf_motor_pin, False)
        self.lpwm.start(self.motor_dut_cycle)

        # Right wheel
        gpio.output(self.rb_motor_pin, True)
        gpio.output(self.rf_motor_pin, False)
        self.rpwm.start(self.motor_dut_cycle)

        counterBR = np.uint64(0)
        counterFL = np.uint64(0)

        buttonBR = int(0)
        buttonFL = int(0)

        i = 0
        while i <= encoder_tics:

            stateBR = gpio.input(self.right_encoder_pin)
            stateFL = gpio.input(self.left_encoder_pin)

            # print(f'counterBR = {counterBR}, counterFL = {counterFL},\
            # GPIO BRstate: {stateBR}, GPIO FLstate: {stateFL}')

            # Save encoder states to txt files
            if self.monitor_encoders is True:
                self.MonitorEncoders('RightPiv', stateBR, stateFL)

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

            # Break when both encoder counts reached the desired total
            if counterBR >= encoder_tics:
                self.rpwm.stop()
                self.lpwm.stop()
                break

    def PID(self, target, present):
        """A function which computes the PID controller output value. 'target'
        is used to store the setpoint and 'present' is used to store the
        current value
        Steps to calculate output :
        1) err is the difference between the target and the present value
        2) The proportional term is Kp times the error
        3) The error is multiplied with the time step dt and added to the
        integral variable
        4) The integral term is Ki times the integral variable
        5) The derivate term is Kd times the difference in present error and
        previous error divided by the time step
        6) Total output is the bounded (withing min and max) sum of the
        proportional, integral, and derivate term

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

    def KeyInput(self, key, value):
        """Operate robot through user input to drive and open/close gripper

        Args:
            event (str): 'w', 's', 'a', 'd', 'o', 'c', 'p' to choose an action
            for the robot
            value (float): Distance to travel/Angle to turn
        """

        print(f'Key: {key}')
        print(f'Distance/Angle: {value}')

        key_press = key

        # Measure distance of objects before following a command
        self.Distance()

        if key_press.lower() == 'w':
            self.Forward(value)
        elif key_press == 's':
            self.Reverse(value)
        elif key_press == 'a':
            self.LeftPiv(value)
        elif key_press == 'd':
            self.RightPiv(value)
        elif key_press == 'o':
            self.OpenGripper()
        elif key_press == 'c':
            self.CloseGripper()
        else:
            print('Invlaid key pressed!!')

    def Teleop(self):
        """Control loop to teleop the robot
        """

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

            value = input("Distance to travel/Angle to turn: ")

            self.KeyInput(key_press, float(value))

            self.GameOver()

    def Navigate(self):

        # Collect navigation commands
        cmds = []
        while True:
            cmd = input('Type command in the format \
                (direction, distance/angle) "q" to finish: ').split(', ')

            if cmd[0] == 'q' or cmd[1] == 'q':
                break

            cmd[1] = float(cmd[1])
            cmds.append(cmd)

        # Unpack commands and control bot
        for key, val in cmds:
            self.KeyInput(key, val)
            time.sleep(1.5)


if __name__ == '__main__':

    robot = Robot(monitor_encoders=False)
    robot.Teleop()
    # robot.Navigate()
    # while True:
    #     init_angle = robot.ReadIMU()
    #     print(f'angle: {init_angle}')
    robot.GameOver()
    gpio.cleanup()
