#!/usr/bin/env python3
import pigpio
import time
from support.Constants import *


class CamTest:
    def __init__(self):
        """
        Initializations
        """
        # Configure the Camera Servo
        self.cam_servo_pin = SERVO_CAM

        pi = pigpio.pi()  # Connect to local Pi.

        pi.set_servo_pulsewidth(self.cam_servo_pin, 1000)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(self.cam_servo_pin, 1500)
        time.sleep(0.5)

        # switch servo off
        pi.set_servo_pulsewidth(self.cam_servo_pin, 0)

        pi.stop()


if __name__ == "__main__":
    ct = CamTest()