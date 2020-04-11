#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
from support.Constants import *


class CamTest:
    def __init__(self):
        """
        Initialization
        """

        # Configure the Camera Servo
        self.cam_servo_pin = SERVO_CAM
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.cam_servo_pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.cam_servo_pin, 50)

        # Move Servo
        self.servo.start(2)  # Start
        time.sleep(.5)  # Wait
        self.servo.stop()  # Stop


if __name__ == "__main__":
    ct = CamTest()
