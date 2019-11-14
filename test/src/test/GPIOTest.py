#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import sys

#Pins
led = 18
switch = 17
interrupt = 23
power = 22

def interrupt_test(channel):
	print("Interrupt detected!")

if __name__ == "__main__":
	if (len(sys.argv)) is 1:
		print("arguements are needed for execution (led, switch, interrupt, ect.)")

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





	#Interrupts

	elif str(sys.argv[1]) == "interrupt":
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(interrupt, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		GPIO.setup(power, GPIO.OUT)
		GPIO.output(power, GPIO. HIGH)
		GPIO.add_event_detect(interrupt, GPIO.FALLING, callback=interrupt_test, bouncetime=300)
		num = 0
		try:
			while(True):
				#Do busy things in here
				for x in range(1, 500):
					num += x
					num %= 2



		except KeyboardInterrupt:
			GPIO.cleanup()

