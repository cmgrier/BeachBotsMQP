#!/usr/bin/env python
import RPi.GPIO as GPIO
from support.Constants import *


class Test:
    def __init__(self):
        """
        Initialization
        """
        switch = SWITCH
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(switch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        while True:
            if GPIO.input(switch) == 1:
                print("PRESSED")
            elif GPIO.input(switch) == 0:
                print("NOT PRESSED")


if __name__ == "__main__":
    test = Test()