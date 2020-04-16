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
import pigpio
import rospy
from std_msgs.msg import Bool
from support.Constants import *


class ArmController:

    def __init__(self):
        """
        Constructor for ArmController class
        """

        # Initialization of the node
        rospy.init_node('ArmController')

        self.pi = pigpio.pi()  # Initialize Pi

        self.kin = Kinematics()
        self.joint0_current = 0.0
        self.joint1_current = 0.0
        self.delay = 0.001

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(SM_DIRECTION, GPIO.OUT)
        GPIO.setup(SM_STEP, GPIO.OUT)

        self.gripper_servo_pin = GRIPPER_SERVO
        self.joint1_pin = JOINT1_SERVO
        self.j1_position = 2000  # Positive is away from camera

        rospy.Subscriber('pickup_flag', Bool, self.pickup_can)
        self.pickup_pub = rospy.Publisher('pickup_done', Bool, queue_size=10)
        self.picking_can = False

        self.calibrate_joints()

    def move_end_effector(self, x, y):
        """
        Move the end effector to the xy-coordinates
        :param x: final x coordinate with respect to the end effector in meters
        :param y: final y coordinate with respect to the end effector in meters
        :return: True if executed properly
        """

        try:
            angles = self.kin.invkin(x, y)
            self.turn_joint0(angles[0])
            self.turn_joint1(angles[1])
            return True
        except ValueError:
            error = 'The location (' + str(x) + ',' + str(y) + ') is out of the task space'
            print(error)
            return False

    def turn_joint0(self, angle):
        """
        Turn the stepper motor for joint 0 to a specific angle
        :param angle: angle of the joint to turn to in degrees
        :return:
        """
        target = angle - self.joint0_current
        steps = abs(int(target // STEP_ANGLE))
        num_steps = 0
        print("target: ", target)
        print("steps: ", steps)

        if target < 0:
            GPIO.output(SM_DIRECTION, GPIO.HIGH)
        else:
            GPIO.output(SM_DIRECTION, GPIO.LOW)
        while num_steps < steps:
            GPIO.output(SM_STEP, GPIO.HIGH)
            rospy.sleep(self.delay)
            GPIO.output(SM_STEP, GPIO.LOW)
            rospy.sleep(self.delay)
            num_steps += 1

    def turn_joint1(self, angle):
        """
        Turn the servo for joint 1 to a specific angle
        :param angle: angle of the joint to turn to in degrees
        :return:
        """
        # TODO

    def calibrate_joints(self):
        """
        Moves the motors until they are zeroed
        :return:
        """

        self.move_gripper(True)
        self.pi.set_servo_pulsewidth(self.joint1_pin, self.j1_position)
        rospy.sleep(3)
        # self.pickup_can(0)

        # trigger = True
        # trigger2 = True
        # # Open gripper
        # # self.gripper_pwm.start(GRIPPER_OPEN)
        # # Zero joint1
        # self.joint1_pwm.start(JOINT1_START)
        #
        # # Zero joint0
        # # Move arm until it triggers the switch
        # GPIO.output(SM_DIRECTION, GPIO.LOW)
        # while trigger:
        #     GPIO.output(SM_STEP, GPIO.HIGH)
        #     rospy.sleep(self.delay)
        #     GPIO.output(SM_STEP, GPIO.LOW)
        #     rospy.sleep(self.delay)
        #     if GPIO.input(SWITCH):
        #         trigger = False
        #         break
        # # Move arm off of switch until it deactivates
        # GPIO.output(SM_DIRECTION, GPIO.HIGH)
        # while trigger2:
        #     GPIO.output(SM_STEP, GPIO.HIGH)
        #     rospy.sleep(self.delay)
        #     GPIO.output(SM_STEP, GPIO.LOW)
        #     rospy.sleep(self.delay)
        #     if not GPIO.input(SWITCH):
        #         trigger2 = False
        #         break

    def move_gripper(self, status):
        """
        Moves the servo for the gripper
        :param status: True opens the gripper and False closes it
        :return:
        """

        if not status:
            self.pi.set_servo_pulsewidth(self.gripper_servo_pin, 2200)
            rospy.sleep(0.5)

        else:
            self.pi.set_servo_pulsewidth(self.gripper_servo_pin, 1300)
            rospy.sleep(0.5)

    def pickup_can(self, msg):
        """
        pickup can callback
        :param msg: Bool
        :return: void
        """

        self.picking_can = True

        while self.picking_can:
            print("Picking Up Can")
            self.move_gripper(True)
            self.pi.set_servo_pulsewidth(self.joint1_pin, 550)
            rospy.sleep(4)
            self.move_gripper(False)
            rospy.sleep(3)
            self.pi.set_servo_pulsewidth(self.joint1_pin, 2000)
            rospy.sleep(3)
            self.move_gripper(True)
            self.pi.set_servo_pulsewidth(self.joint1_pin, self.j1_position)
            self.picking_can = False

        bool_msg = True

        self.pickup_pub.publish(bool_msg)

    def cleanup(self):
        """
        Cleans up node when closes occur
        :return: void
        """
        self.pi.set_servo_pulsewidth(self.joint1_pin, 0)
        self.pi.set_servo_pulsewidth(self.gripper_servo_pin, 0)

        self.pi.stop()


if __name__ == "__main__":
    ac = ArmController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        ac.cleanup()
