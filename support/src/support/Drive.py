#!/usr/bin/env python



import rospy
import roslib
import RPiGPIO as GPIO 

class Drive:
	def __init__(self, l_pin, r_pin, l_direct_1, l_direct_2, r_direct_1, r_direct_2):
		self.l_wheel_pin = l_pin
		self.r_wheel_pin = r_pin
		self.l_direct_1 = l_direct_1
		self.l_direct_2 = l_direct_2 
		self.r_direct_1 = r_direct_1
		self.r_direct_2 = r_direct_2
		
		GPIO.setMode(GPIO.BCM)
		GPIO.setup(self.l_direct_1, GPIO.OUT)
		GPIO.setup(self.l_direct_2, GPIO.OUT)
		GPIO.setup(self.r_direct_1, GPIO.OUT)
		GPIO.setup(self.r_direct_2, GPIO.OUT)
		GPIO.setup(self.l_wheel_pin, GPIO.OUT)
		GPIO.setup(self.r_wheel_pin, GPIO.OUT)
		self.set_direction("F", "F")
		
		self.l_pwm = GPIO.PWM(self.l_wheel_pin, 0)
		self.r_pwm = GPIO.PWM(self.r_wheel_pin, 0)
		
		
	def run_wheels(self,l_freq,r_freq):
		"""
		Begins the PWM for both wheels
		:param l_freq: left wheel pwm varaible
		:param r_freq: right wheel pwm variable
		:return: void
		"""
		
		self.l_pwm = GPIO.PWM(self.l_wheel_pin, l_freq)
		self.r_pwm = GPIO.PWM(self.r_wheel_pin, r_freq)
		self.l_pwm.start(50)
		self.r_pwm.start(50)
	
	def stop_wheels(self):
		"""
		Stops both the wheels
		:return: void
		"""
		self.l_pwm.stop()
		self.r_pwm.stop()
		

	def set_direction(self, lwheel, rwheel):
		"""
		Sets the direction of the left and right wheels
		:param lwheel: direction for left wheel
		:param rhweel: direction for right wheel
		:return:
		"""
		
		if lwheel == "F":
			GPIO.output(self.l_direct_1, GPIO.HIGH)
			GPIO.output(self.l_direct_2, GPIO.LOW)

		elif lwheel == "B":
 			GPIO.output(self.l_direct_1, GPIO.LOW)
			GPIO.output(self.l_direct_2, GPIO.HIGH)

		if rwheel == "F":
			GPIO.output(self.r_direct_1, GPIO.LOW)
			GPIO.output(self.r_direct_2, GPIO.HIGH)

		elif rwheel == "B":
			GPIO.output(self.r_direct_1, GPIO.HIGH)
			GPIO.output(self.r_direct_2, GPIO.LOW)

	def cleanup(self):
		self.stop_wheels()
		GPIO.cleanup()
	#TODO add PID capability
		
