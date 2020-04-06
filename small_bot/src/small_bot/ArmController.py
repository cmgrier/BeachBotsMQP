#!/usr/bin/python
# title           :ArmController.py
# description     :smallbot arm controller
# author          :Sean Tidd
# date            :2020-02-11
# version         :0.1
# notes           :
# python_version  :3.5
# ==============================================================================
from small_bot.Kinematics import Kinematics
import RPi.GPIO as GPIO
import rospy
from support.Constants import *


class ArmController:

    def __init__(self):
        """
        Constructor for ArmController class
        """
        self.kin = Kinematics()
        self.joint0_current = 0.0
        self.joint1_current = 0.0
        self.delay = 0.001
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(COIL_A_1_PIN, GPIO.OUT)
        GPIO.setup(COIL_A_2_PIN, GPIO.OUT)
        GPIO.setup(COIL_B_1_PIN, GPIO.OUT)
        GPIO.setup(COIL_B_2_PIN, GPIO.OUT)
        GPIO.setup(JOINT1_SERVO, GPIO.OUT)
        GPIO.setup(GRIPPER_SERVO, GPIO.OUT)
        self.joint1_pwm = GPIO.PWM(JOINT1_SERVO, 50)
        self.gripper_pwm = GPIO.PWM(GRIPPER_SERVO, 50)
        self.turn_joint0(-5)

    def move_end_effector(self, x, y):
        """
        Move the end effector to the xy-coordinates
        :param x: final x coordinate with respect to the end effector in meters
        :param y: final y coordinate with respect to the end effector in meters
        :return: True if executed properly
        """

        try:
            angles = self.kin.invkin(x,y)
            self.turn_joint0(angles[0])
            self.turn_joint1(angles[1])
            return True
        except ValueError:
            error = 'The location ('+str(x)+','+str(y)+') is out of the task space'
            print(error)
            return False

    def set_step(self,w1, w2, w3, w4):
        """
        Set the voltage for the stepper motor coils
        :param w1: coil A pin 1 voltage
        :param w2: coil A pin 2 voltage
        :param w3: coil B pin 1 voltage
        :param w4: coil B pin 2 voltage
        :return:
        """
        GPIO.output(COIL_A_1_PIN, w1)
        GPIO.output(COIL_A_2_PIN, w2)
        GPIO.output(COIL_B_1_PIN, w3)
        GPIO.output(COIL_B_2_PIN, w4)

    def turn_joint0(self, angle):
        """
        Turn the stepper motor for joint 0 to a specific angle
        :param angle: angle of the joint to turn to in degrees
        :return:
        """
        target = angle - self.joint0_current
        steps = abs(int(target//STEP_ANGLE))/4
	print("target: ",target)
	print("steps: ",steps)

	rospy.sleep(self.delay)
        if target < 0:
            for i in range(0, steps):
                self.set_step(1, 0, 1, 0)
                rospy.sleep(self.delay)
                self.set_step(0, 1, 1, 0)
                rospy.sleep(self.delay)
                self.set_step(0, 1, 0, 1)
                rospy.sleep(self.delay)
                self.set_step(1, 0, 0, 1)
                rospy.sleep(self.delay)

        else:
            # Reverse previous step sequence to reverse motor direction

            for i in range(0, steps):
                self.set_step(1, 0, 0, 1)
                rospy.sleep(self.delay)
                self.set_step(0, 1, 0, 1)
                rospy.sleep(self.delay)
                self.set_step(0, 1, 1, 0)
                rospy.sleep(self.delay)
                self.set_step(1, 0, 1, 0)
                rospy.sleep(self.delay)
	self.set_step(0,0,0,0)



    def turn_joint1(self, angle):
        """
        Turn the servo for joint 1 to a specific angle
        :param angle: angle of the joint to turn to in degrees
        :return:
        """
        print("joimt1: ",angle)
        duty =   (angle * 0.035) + JOINT1_START
	if duty < 0:
	 duty = 0
        GPIO.output(JOINT1_SERVO, True)
        self.joint1_pwm.ChangeDutyCycle(duty)
        rospy.sleep(1)
        GPIO.output(JOINT1_SERVO, False)
        self.joint1_pwm.ChangeDutyCycle(0)

    def calibrate_joints(self):
        """
        Moves the motors until they are zeroed
        :return:
        """
        trigger = True
	trigger2 = True
        #Open gripper
        self.gripper_pwm.start(GRIPPER_OPEN)
        #Zero joint1
        self.joint1_pwm.start(JOINT1_START)

        #Zero joint0
        while trigger:
	   self.set_step(1, 0, 0, 1)
           rospy.sleep(self.delay)
           self.set_step(0, 1, 0, 1)
           rospy.sleep(self.delay)
           self.set_step(0, 1, 1, 0)
           rospy.sleep(self.delay)
           self.set_step(1, 0, 1, 0)
           rospy.sleep(self.delay)
	   if GPIO.input(SWITCH):
		trigger = False
		break
	self.set_step(0,0,0,0)
	while trigger2:
           self.set_step(1, 0, 1,0 )
           rospy.sleep(self.delay)
           self.set_step(0, 1, 1, 0)
           rospy.sleep(self.delay)
           self.set_step(0, 1, 0, 1)
           rospy.sleep(self.delay)
           self.set_step(1, 0, 0, 1)
           rospy.sleep(self.delay)
           if not GPIO.input(SWITCH):
                trigger2 = False
                break  
	self.set_step(0,0,0,0)

    def move_gripper(self, status):
        """
        Moves the servo for the gripper
        :param status: True opens the gripper and False closes it
        :return:
        """
        if status == False:
            GPIO.output(GRIPPER_SERVO, True)
            self.gripper_pwm.ChangeDutyCycle(GRIPPER_CLOSE)
            rospy.sleep(1)
            GPIO.output(GRIPPER_SERVO, False)
            self.gripper_pwm.ChangeDutyCycle(0)
        else:
            GPIO.output(GRIPPER_SERVO, True)
            self.gripper_pwm.ChangeDutyCycle(GRIPPER_OPEN)
            rospy.sleep(1)
            GPIO.output(GRIPPER_SERVO, False)
            self.gripper_pwm.ChangeDutyCycle(0)



if __name__=="__main__":
    arm = ArmController()
    try:
     arm.turn_joint0(45)
     GPIO.cleanup()
    except KeyboardInterrupt:
	 GPIO.cleanup()
