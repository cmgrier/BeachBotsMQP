#!/usr/bin/env python
import maestro
import rospy
import time
from small_bot.src.small_bot.ServoController import ServoController

class Strength_Test:
    def __init__(self):
        self.controller = ServoController()

    def Test1(self):
        self.controller.go_to_position(2000, 4000)
        self.servo.close()


# Testing
Test = Strength_Test()
Test.Test1()
