#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import sys

#Pins
led = 18
switch = 17


if __name__ == "__main__":
	if (len(sys.argv)) is 1:
		print("arguements are needed for execution (led, switch, ect.)")

	#Light up an LED
	elif str(sys.argv[1]) == "led":
		try:
			GPIO.setmode(GPIO.BCM)
			GPIO.setup(led,GPIO.OUT)
			GPIO.output(led, GPIO.HIGH)
			rospy.sleep(5)
			GPIO.output(led, GPIO.LOW)
			GPIO.cleanup()
		
		except KeyboardInterrupt:
			GPIO.cleanup()
	

	#Read input from a switch
	elif str(sys.argv[1]) == "switch":
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(switch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		try:
			while(True):
				if GPIO.input(switch) == 1:
					print("PRESSED")
				elif GPIO.input(switch) == 0:
					print("NOT PRESSED")
		except KeyboardInterrupt:
			GPIO.cleanup()
		

	#Test PWM generation	
	

