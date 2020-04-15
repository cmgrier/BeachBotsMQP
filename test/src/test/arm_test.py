#!/usr/bin/python
# title		:arm_test.py
# description	:test basic smallbot arm functionality
# author	:Sean Tidd	
# date		:2020-04-5
# version	:0.1
# notes		:
# python_version :3.5
# =============================================================================
from small_bot.ArmController import ArmController
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
	
	def test_can_grab(self):
	     #rospy.sleep(3)
	     #self.arm.turn_joint1(-35)
	     rospy.sleep(3)
	     self.arm.move_gripper(False)
             rospy.sleep(9)
	     #self.arm.turn_joint1(0)
	     #rospy.sleep(3)		


if __name__ == "__main__":
	try:
	   armTest = ArmTest()
	   armTest.test_can_grab()
	except KeyboardInterrupt:
	  GPIO.cleanup()	
     
