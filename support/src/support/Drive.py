#!/usr/bin/env python



import rospy
import roslib
import RPi.GPIO as GPIO 
from support.Constants import *
import geometry_msgs.msg

class Drive:
	def __init__(self):
		self.l_wheel_pin = SMALL_L_WHEEL_PIN
		self.r_wheel_pin = SMALL_R_WHEEL_PIN
		self.l_direct_1 = SMALL_L_DIRECT_1
		self.l_direct_2 = SMALL_L_DIRECT_2
		self.r_direct_1 = SMALL_R_DIRECT_1
		self.r_direct_2 = SMALL_R_DIRECT_2

		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.l_direct_1, GPIO.OUT)
		GPIO.setup(self.l_direct_2, GPIO.OUT)
		GPIO.setup(self.r_direct_1, GPIO.OUT)
		GPIO.setup(self.r_direct_2, GPIO.OUT)
		GPIO.setup(self.l_wheel_pin, GPIO.OUT)
		GPIO.setup(self.r_wheel_pin, GPIO.OUT)
		self.set_direction("F", "F")
		
		self.l_pwm = GPIO.PWM(self.l_wheel_pin, 200)
		self.r_pwm = GPIO.PWM(self.r_wheel_pin, 200)
		
	def listener(self):
		rospy.init_node('drive_listener', anonymous=True)
                rospy.Subscriber("/cmd_vel", geometry_msgs.msg.Twist, self.interpreter)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

	def interpreter(self, msg):
		print("In CALLBACK")
	
		#Detected angular velocity
		if msg.angular.z != 0.0 and msg.linear.x == 0.0:
			print("IN ANGULAR ")
			val = (msg.angular.z * 100)
			if val > 100:
				val = 100
			elif val < -100:
				val = -100

			if val >= 0:
				self.set_direction("B", "F")
			elif val < 0:
				self.set_direction("F", "B")
				val *= -1
			self.run_wheels(val, val)

		#Detected linear velocity
		elif msg.angular.z == 0.0 and msg.linear.x != 0.0:
			print("In LINEAR: ",msg.linear.x)
			val = (msg.linear.x * 100)
			if val > 100:
				val = 100
			elif val < -100:
				val = -100
			if val >= 0:
				self.set_direction("F", "F")
			elif val < 0:
				self.set_direction("B", "B")
				val *= -1
			self.run_wheels(val, val)

		#No speed detected
		else:
			self.stop_wheels()

		"""else:
			turn_val = msg.angular.z
			lin_val = msg.linear.x

			right_wheel_val = lin_val - ((turn_val / lin_val) * 25)
			left_wheel_val = lin_val + ((turn_val / lin_val) * 25)

			if left_wheel_val < 0:
				left_wheel_val = math.abs(left_wheel_val)
			if right_wheel_val < 0:
				right_wheel_val = math.abs(right_wheel_val)

			run_wheels(left_wheel_val, right_wheel_val)
		"""



	def run_wheels(self,l_speed,r_speed):
		"""
		Begins the PWM for both wheels
		:param l_speed: left wheel duty cycle variable
		:param r_speed: right wheel duty cycle variable
		:return: void
		"""
		
		self.l_pwm.start(float(l_speed))
		self.r_pwm.start(float(r_speed))
	
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


if __name__ == "__main__":
	drive = Drive()
	try:
		drive.listener()
	except KeyboardInterrupt:
		drive.cleanup()
