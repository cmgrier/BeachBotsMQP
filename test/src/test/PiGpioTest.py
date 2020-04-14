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
        self.cam_servo_pin = GRIPPER_SERVO

        pi = pigpio.pi()  # Connect to local Pi.

        pi.set_PWM_frequency(self.cam_servo_pin, 40000)
        # pi.set_PWM_dutycycle(self.cam_servo_pin, 255)
        # pi.set_pull_up_down(self.cam_servo_pin, pigpio.PUD_UP)
        pi.set_servo_pulsewidth(self.cam_servo_pin, 1500)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(self.cam_servo_pin, 2100)
        time.sleep(0.5)

        # switch servo off
        # pi.set_servo_pulsewidth(self.cam_servo_pin, 0)

        pi.stop()


if __name__ == "__main__":
    ct = CamTest()
