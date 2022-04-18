
import RPi.GPIO as gpio
import time
import numpy as np
import serial
import cv2
import imutils


class Robot:

    def __init__(self, monitor_encoders=False,
                 monitor_imu=False,
                 debug_mode=False):

        # Save Encoder data to text files
        self.monitor_encoders = monitor_encoders

        self.debug_mode = debug_mode

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
        self.monitor_imu = monitor_imu
        self.imu_angle = 0
        self.imu_margin = 3

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
        self.wheel_base = 0.1397  # Meters (5.5")
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

    def MonitorIMU(self, updated_angle):
        # Save encoder states to txt file
        file = open(f'imu_data.txt', 'a')
        # Save encoder states to txt files
        outstring = str(updated_angle) + '\n'
        file.write(outstring)

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

    def ReadIMU(self, count):
        """Read IMU data and return the value as a float

        Args:
            count (int): Number of times IMU reading has been read

        Returns:
            float: x-axis orientation value
              int: Number of times IMU reading has been read
        """

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

        else:
            line = 0

        return line, count

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
        encoder_tics = int(self.drive_constant * distance)

        # Get Initial IMU angle reading
        init_angle = self.imu_angle

        if init_angle > 359:
            init_angle -= 360

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

        print(f'Duty Cycle: {self.motor_dut_cycle}')

        count = 0
        while True:

            if self.ser.in_waiting > 0:
                updated_angle, count = self.ReadIMU(count)

            # if 180 < updated_angle < 360:
            #     updated_angle -= 360

            stateBR = gpio.input(self.right_encoder_pin)
            stateFL = gpio.input(self.left_encoder_pin)

            # Save encoder states to txt files
            if self.monitor_encoders is True:
                self.MonitorEncoders('Forward', stateBR, stateFL)

            if self.monitor_imu is True:
                self.MonitorIMU(updated_angle)

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
            imu_margin = 1
            low_thresh = init_angle - imu_margin
            high_thresh = init_angle + imu_margin

            if low_thresh < 0:
                low_thresh += 360

            if high_thresh > 359:
                high_thresh -= 360
            # print(f'Goal: {encoder_tics} R: {counterBR} L: {counterFL}\
            #  Angle: {updated_angle} Dutycycle: {self.motor_dut_cycle}')

            if self.debug_mode:
                print(f'Angle: {updated_angle} Initial: {init_angle}\
                    Dutycycle: {self.motor_dut_cycle}')

            if updated_angle < low_thresh:

                # Double speed to match encoder counts
                speed_update = min(self.motor_dut_cycle * 2, 100)
                self.lpwm.ChangeDutyCycle(speed_update)

            if updated_angle > high_thresh:

                # Double speed to match encoder counts
                speed_update = min(self.motor_dut_cycle * 2, 100)
                self.rpwm.ChangeDutyCycle(speed_update)

            if low_thresh < updated_angle < high_thresh:

                self.rpwm.ChangeDutyCycle(self.motor_dut_cycle)
                self.lpwm.ChangeDutyCycle(self.motor_dut_cycle)

            # Break when both encoder counts reached the desired total
            if counterBR >= encoder_tics and counterFL >= encoder_tics:
                self.rpwm.stop()
                self.lpwm.stop()
                self.imu_angle = updated_angle
                break

    def Reverse(self, distance):
        """Move robot in reverse for 'distance' meters

        Args:
            distance (int): Distance in meters for robot to drive in reverse
        """

        # Total encoder tics to drive the desired distance
        encoder_tics = int(self.drive_constant * distance)

        # Get Initial IMU angle reading
        init_angle = 0

        # Left wheel
        gpio.output(self.lb_motor_pin, False)
        gpio.output(self.lf_motor_pin, True)
        self.lpwm.start(self.motor_dut_cycle)

        # Right wheel
        gpio.output(self.rb_motor_pin, True)
        gpio.output(self.rf_motor_pin, False)
        self.rpwm.start(self.motor_dut_cycle)

        counterBR = np.uint64(0)
        counterFL = np.uint64(0)

        buttonBR = int(0)
        buttonFL = int(0)

        print(f'Duty Cycle: {self.motor_dut_cycle}')

        count = 0
        while True:

            if self.ser.in_waiting > 0:
                updated_angle, count = self.ReadIMU(count)

            if 300 < updated_angle < 360:
                updated_angle -= 360

            stateBR = gpio.input(self.right_encoder_pin)
            stateFL = gpio.input(self.left_encoder_pin)

            # Save encoder states to txt files
            if self.monitor_encoders is True:
                self.MonitorEncoders('Reverse', stateBR, stateFL)

            if self.monitor_imu is True:
                self.MonitorIMU(updated_angle)

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
            imu_margin = 0.5
            low_thresh = init_angle - imu_margin
            high_thresh = init_angle + imu_margin

            if self.debug_mode:
                print(f'Goal: {encoder_tics} R: {counterBR} L: {counterFL}\
                    Angle: {updated_angle} Dutycycle: {self.motor_dut_cycle}')

            if updated_angle < low_thresh:

                # Double speed to match encoder counts
                speed_update = min(self.motor_dut_cycle * 2, 100)
                self.rpwm.ChangeDutyCycle(speed_update)

            if updated_angle > high_thresh:

                # Double speed to match encoder counts
                speed_update = min(self.motor_dut_cycle * 2, 100)
                self.lpwm.ChangeDutyCycle(speed_update)

            if low_thresh < updated_angle < high_thresh:

                self.rpwm.ChangeDutyCycle(self.motor_dut_cycle)
                self.lpwm.ChangeDutyCycle(self.motor_dut_cycle)

            # Break when both encoder counts reached the desired total
            if counterBR >= encoder_tics and counterFL >= encoder_tics:
                self.rpwm.stop()
                self.lpwm.stop()
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

        # Get Initial IMU angle reading
        init_angle = self.imu_angle

        if init_angle < 0:
            init_angle += 360

        goal_angle = init_angle - angle

        if goal_angle < 0:
            goal_angle += 360

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

        count = 0
        while True:

            if self.ser.in_waiting > 0:
                updated_angle, count = self.ReadIMU(count)

            # if 180 < updated_angle < 360:
            #     updated_angle = 360 - updated_angle

            if count > 10:  # Ignore the first 10 IMU readings

                stateBR = gpio.input(self.right_encoder_pin)
                stateFL = gpio.input(self.left_encoder_pin)

                # print(f'counterBR = {counterBR}, counterFL = {counterFL}, \
                # GPIO BRstate: {stateBR}, GPIO FLstate: {stateFL}')

                # Save encoder states to txt files
                if self.monitor_encoders is True:
                    self.MonitorEncoders('LeftPiv', stateBR, stateFL)

                if self.monitor_imu is True:
                    self.MonitorIMU(updated_angle)

                if int(stateBR) != int(buttonBR):
                    buttonBR = int(stateBR)
                    counterBR += 1

                if int(stateFL) != int(buttonFL):
                    buttonFL = int(stateFL)
                    counterFL += 1

                # if counterBR >= encoder_tics:
                #     self.rpwm.stop()

                # if counterFL >= encoder_tics:
                #     self.lpwm.stop()

                if self.debug_mode:
                    print(f'Goal: {goal_angle} Angle: {updated_angle}\
                        Initial: {init_angle} Dutycycle: {self.motor_dut_cycle}')

                # PID tunning
                low_thresh = goal_angle - self.imu_margin
                high_thresh = goal_angle + self.imu_margin

                # Break when the current angle is within a threshold of the
                # goal angle
                if low_thresh < updated_angle < high_thresh:

                    # Left wheel
                    gpio.output(self.lb_motor_pin, False)
                    gpio.output(self.lf_motor_pin, False)
                    self.lpwm.stop()

                    # Right wheel
                    gpio.output(self.rb_motor_pin, False)
                    gpio.output(self.rf_motor_pin, False)
                    self.rpwm.stop()

                    self.imu_angle = updated_angle
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

        # Get Initial IMU angle reading
        init_angle = self.imu_angle

        if init_angle < 0:
            init_angle += 360

        goal_angle = init_angle + angle

        if goal_angle > 360:
            goal_angle -= 360

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

        count = 0
        while True:

            if self.ser.in_waiting > 0:
                updated_angle, count = self.ReadIMU(count)

            if count > 10:  # Ignore the first 10 IMU readings

                stateBR = gpio.input(self.right_encoder_pin)
                stateFL = gpio.input(self.left_encoder_pin)

                # print(f'counterBR = {counterBR}, counterFL = {counterFL},\
                # GPIO BRstate: {stateBR}, GPIO FLstate: {stateFL}')

                # Save encoder states to txt files
                if self.monitor_encoders is True:
                    self.MonitorEncoders('RightPiv', stateBR, stateFL)

                if self.monitor_imu is True:
                    self.MonitorIMU(updated_angle)

                if int(stateBR) != int(buttonBR):
                    buttonBR = int(stateBR)
                    counterBR += 1

                if int(stateFL) != int(buttonFL):
                    buttonFL = int(stateFL)
                    counterFL += 1

                # if counterBR >= encoder_tics:
                #     self.rpwm.stop()

                # if counterFL >= encoder_tics:
                #     self.lpwm.stop()

                # PID tunning
                low_thresh = goal_angle - self.imu_margin
                high_thresh = goal_angle + self.imu_margin

                if self.debug_mode:
                    print(f'Goal: {goal_angle} Angle: {updated_angle}\
                        Initial: {init_angle} Dutycycle: {self.motor_dut_cycle}')

                # Break when the current angle is within a threshold of the
                # goal angle
                if low_thresh < updated_angle < high_thresh:

                    # Left wheel
                    gpio.output(self.lb_motor_pin, False)
                    gpio.output(self.lf_motor_pin, False)
                    self.lpwm.stop()

                    # Right wheel
                    gpio.output(self.rb_motor_pin, False)
                    gpio.output(self.rf_motor_pin, False)
                    self.rpwm.stop()

                    self.imu_angle = updated_angle
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

    def ColorRange(self, color='green'):

        if color == 'red':
            low_red = np.array([0, 34, 181])
            high_red = np.array([179, 140, 255])
            return low_red, high_red

        elif color == 'green':
            low_green = np.array([50, 100, 70])
            high_green = np.array([90, 255, 255])
            return low_green, high_green

        elif color == 'yellow':
            low_yellow = np.array([0, 134, 134])
            high_yellow = np.array([179, 255, 255])
            return low_yellow, high_yellow

    def DistFromCenter(self, x_pos):

        midline = 640 / 2

        pix_per_deg = 38.88 / 640

        pos = x_pos - midline

        if pos > 0:
            angle = pix_per_deg * pos
            self.RightPiv(angle)
            direction = 'Right'
        else:
            angle = pix_per_deg * abs(pos)
            self.LeftPiv(angle)
            direction = 'Left'

        print(f'Angle to Turn {direction}: {angle}')

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


def TrackColor(robot):
    cap = cv2.VideoCapture(0)

    ret, frame = cap.read()
    if ret:    # frame captured without any errors

        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_thresh, high_thresh = robot.ColorRange()
        mask = cv2.inRange(hsv, low_thresh, high_thresh)
        # color_mask = cv2.bitwise_and(frame, frame, mask=mask)

        # Find the contours
        cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # Draw contours and track center
        for c in cnts:

            # (x, y), radius = cv2.minEnclosingCircle(c)
            # center = (int(x), int(y))
            # radius = int(radius)

            rect = cv2.boundingRect(c)
            min_box_size = 15
            if rect[2] < min_box_size or rect[3] < min_box_size:
                continue

            x, y, w, h = rect

            center = (int(x + w / 2), int(y + h / 2))

            cv2.rectangle(frame, (x, y), (x + w, y + h),
                          color=(0, 255, 255), thickness=2)

            robot.DistFromCenter(x)

            # cv2.circle(frame, center, radius,
            #            color=(0, 255, 255), thickness=2)
            cv2.circle(frame, center, 1, color=(0, 0, 255), thickness=4)

        # The original input frame is shown in the window
        cv2.imshow('Original', frame)
        cv2.waitKey(0)

    # while(True):

    #     # reads frames from a camera
    #     # ret checks return at each frame
    #     ret, frame = cap.read()

    #     frame = cv2.flip(frame, 0)

    #     if not ret:
    #         break

    #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #     low_thresh, high_thresh = robot.ColorRange()
    #     mask = cv2.inRange(hsv, low_thresh, high_thresh)
    #     # color_mask = cv2.bitwise_and(frame, frame, mask=mask)

    #     # Find the contours
    #     cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #     cnts = imutils.grab_contours(cnts)

    #     # Draw contours and track center
    #     for c in cnts:

    #         # (x, y), radius = cv2.minEnclosingCircle(c)
    #         # center = (int(x), int(y))
    #         # radius = int(radius)

    #         rect = cv2.boundingRect(c)
    #         if rect[2] < 50 or rect[3] < 50:
    #             continue

    #         x, y, w, h = rect

    #         center = (int(x + w / 2), int(y + h / 2))

    #         cv2.rectangle(frame, (x, y), (x + w, y + h),
    #                       color=(0, 255, 255), thickness=2)

    #         robot.DistFromCenter(x)

    #         # cv2.circle(frame, center, radius,
    #         #            color=(0, 255, 255), thickness=2)
    #         cv2.circle(frame, center, 1, color=(0, 0, 255), thickness=4)

    #     # The original input frame is shown in the window
    #     cv2.imshow('Original', frame)
    #     # cv2.imshow('Green Mask', mask)
    #     # compare_frame = np.hstack((frame, hsv,
    #     #                            cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
    #     # cv2.imshow('hstack: ', compare_frame)

    #     # Wait for 'a' key to stop the program
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break

    # Close the window / Release webcam
    cap.release()

    # De-allocate any associated memory usage
    cv2.destroyAllWindows()


if __name__ == '__main__':

    robot = Robot(monitor_encoders=False, monitor_imu=False)

    for i in range(10):
        TrackColor(robot)
    # robot.Teleop()
    # robot.Navigate()
    # count = 0
    # while True:
    #     # init_angle, count = robot.ReadIMU(count)
    #     print(f'angle: {robot.imu_angle}')

    robot.GameOver()
    gpio.cleanup()
