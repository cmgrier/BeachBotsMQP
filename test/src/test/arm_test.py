#!/usr/bin/python
# title		:arm_test.py
# description	:test basic smallbot arm functionality
# author	:Sean Tidd	
# date		:2020-04-5
# version	:0.1
# notes		:
# python_version :3.5
# =============================================================================
#from small_bot.collector_arm.ArmController import ArmController
import RPi.GPIO as GPIO
import rospy
from support.Constants import *

def init():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(SWITCH,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	

if __name__ == "__main__":
	try:
	   init()
           while True:
	      if GPIO.input(SWITCH):
	         print("3.3V")
	      else:
		 print("0.0V")
	except KeyboardInterrupt:
	  GPIO.cleanup()	
