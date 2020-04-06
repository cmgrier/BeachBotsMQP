#!/usr/bin/python
# title		:arm_test.py
# description	:test basic smallbot arm functionality
# author	:Sean Tidd	
# date		:2020-04-5
# version	:0.1
# notes		:
# python_version :3.5
# =============================================================================
import sys
sys.path.insert(1,'/catkin_ws/src/BeachCleanersMQP/small_bot/src/small_bot/collector_arm/ArmController')
import ArmController
import RPi.GPIO as GPIO
import rospy
from support.Constants import *

class ArmTest:
	def __init__(self):
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(SWITCH,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		self.arm = ArmController()

	def test_switch(self):
	     if GPIO.input(SWITCH):
                 print("3.3V")
             else:
                 print("0.0V")
		

if __name__ == "__main__":
	try:
	   armTest = ArmTest()
           #while True:
	    #  armTest.test_switch()
	except KeyboardInterrupt:
	  GPIO.cleanup()	
     
