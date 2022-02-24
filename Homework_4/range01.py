import RPi.GPIO as gpio
import time
import cv2
import imutils
import os

def Distance(trig, echo):

    # Assign inputs and outputs to RPi GPIO pins
    gpio.setmode(gpio.BOARD)
    gpio.setup(trig, gpio.OUT)
    gpio.setup(echo, gpio.IN)

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
    return distance

def RaspImg(name):
    os.system(f'raspistill -w 640 -h 480 -o {name}')

if __name__ == '__main__':

    # Take Image of scene
    name = 'lecture4inclass.jpg'
    RaspImg(name)

    # Define pin allocations
    trig = 16
    echo = 18

    # Take 10 successive range measurements at a rate of 1Hz
    dis_list = []
    for i in range(10):
        dis_list.append(Distance(trig, echo))
        print(f'Distance: {Distance(trig, echo)} cm')
        time.sleep(1)

    # Calculate average distance recorded
    avg_dis = round(sum(dis_list) / len(dis_list), 2)
    print(f'Average Distance: {avg_dis}')

    # Write the average distance on the image
    img = cv2.imread(name)
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (20, 40)
    font_scale = 1
    font_color = (0, 0, 255)
    thickness = 2
    cv2.putText(img, f'Average Distance: {avg_dis}',org, font, font_scale, font_color, thickness)
    cv2.imshow('image', img)
    cv2.waitKey(0)

    # Filename
    filename = 'homework4.jpg'
    
    # Using cv2.imwrite() method
    # Saving the image
    cv2.imwrite(filename, img)
