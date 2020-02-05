#!/usr/bin/python
from small_bot.collector_arm.Kinematics import Kinematics
import RPi.GPIO as GPIO
import rospy
from support.Constants import *
import time

class ArmController:

    def __init__(self):
        """
        Constructor for ArmController class
        :param pin0: pin for joint 0 stepper motor
        :param pin1: pin for joint 1 servo
        :param pin2: pin for gripper servo
        """
        self.kin = Kinematics()
        self.joint0_current = 0.0
        self.joint1_current = 0.0
        self.delay = 0.001
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(DIR, GPIO.OUT)
        GPIO.setup(STEP, GPIO.OUT)
        GPIO.setup(SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        GPIO.setup(COIL_A_1_PIN, GPIO.OUT)
        GPIO.setup(COIL_A_2_PIN, GPIO.OUT)
        GPIO.setup(COIL_B_1_PIN, GPIO.OUT)
        GPIO.setup(COIL_B_2_PIN, GPIO.OUT)

        #self.calibrate_joints()

    def move_end_effector(self, x, y):
        """
        Move the end effector to the xy-coordinates
        :param x: final x coordinate with respect to the end effector in meters
        :param y: final y coordinate with respect to the end effector in meters
        :return: True if executed properly
        """
        #TODO
        try:
            angles = self.kin.invkin(x,y)
            self.turn_joint0(angles[0])
            self.turn_joint1(angles[1])
            return True
        except ValueError:
            error = 'The location ('+str(x)+','+str(y)+') is out of the task space'
            print(error)
            return False

    def setStep(self,w1, w2, w3, w4):
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

        if target < 0:
            for i in range(0, steps):
                self.setStep(1, 0, 1, 0)
                rospy.sleep(self.delay)
                self.setStep(0, 1, 1, 0)
                rospy.sleep(self.delay)
                self.setStep(0, 1, 0, 1)
                rospy.sleep(self.delay)
                self.setStep(1, 0, 0, 1)
                rospy.sleep(self.delay)

        else:
            # Reverse previous step sequence to reverse motor direction

            for i in range(0, steps):
                self.setStep(1, 0, 0, 1)
                rospy.sleep(self.delay)
                self.setStep(0, 1, 0, 1)
                rospy.sleep(self.delay)
                self.setStep(0, 1, 1, 0)
                rospy.sleep(self.delay)
                self.setStep(1, 0, 1, 0)
                rospy.sleep(self.delay)

        """
        if target < 0:
            GPIO.output(DIR, CW)
        else:
            GPIO.output(DIR, CCW)
        for step in range(int(target/STEP_ANGLE)):
            GPIO.output(STEP, GPIO.HIGH)
            sleep(self.delay)
            GPIO.output(STEP,GPIO.LOW)
            sleep(self.delay)
        self.joint0_current = angle
        """

    def turn_joint1(self, angle):
        """
        Turn the servo for joint 1 to a specific angle
        :param angle: angle of the joint to turn to in degrees
        :return:
        """
        print("joimt1: ",angle)
        #TODO

    def calibrate_joints(self):
        """
        Moves the motors until they are zeroed
        :return:
        """
        #TODO
        #Zero joint0
        GPIO.output(DIR, CW)
        trigger = True
        while trigger:
            GPIO.output(STEP, GPIO.HIGH)
            sleep(self.delay)
            GPIO.output(STEP,GPIO.LOW)
            sleep(self.delay)
            if GPIO.input(SWITCH):
                trigger = False


    def close_gripper(self, status):
        """
        Moves the servo for the gripper
        :param status: True closes the gripper and False opens it
        :return:
        """
        #TODO


if __name__=="__main__":
    arm = ArmController()
    #arm.move_end_effector(40,0)
    try:
	 arm.turn_joint0(90)
   	 GPIO.cleanup()
    except KeyboardInterrupt:
	 GPIO.cleanup()
