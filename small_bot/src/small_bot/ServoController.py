#!/usr/bin/env python
import maestro
import rospy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Bool
from support.Constants import *


class ServoController:
    def __init__(self):
        # Initialization of the node
        rospy.init_node('ServoController')

        self.servo = maestro.Controller()

        # Pins
        self.gripper_pin = GRIPPER
        self.elbow_pin = ELBOW

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(SM_DIRECTION, GPIO.OUT)
        GPIO.setup(SM_STEP, GPIO.OUT)
        self.delay = 0.001

        # ROS Subscribers and Publishers
        rospy.Subscriber('pickup_flag', Bool, self.pickup_can)
        self.pickup_pub = rospy.Publisher('pickup_done', Bool, queue_size=10)

        self.picking_can = False

        # Run Calibration of Joints
        self.calibrate()

    def calibrate(self):
        """
        Calibrates all servos
        """
        # Move the joint to over the bucket
        self.elbow(8500)
        # Open the Gripper
        self.gripper(True)
        # Stepper Motor
        rospy.sleep(2)
        self.stepper_motor()

    def stepper_motor(self):
        """
        Handles the Stepper motor
        """
        trigger = True
        trigger2 = True

        # Zero joint0
        # Move arm until it triggers the switch
        GPIO.output(SM_DIRECTION, GPIO.LOW)
        while trigger:
            # print("Made it into step loop")
            GPIO.output(SM_STEP, GPIO.HIGH)
            time.sleep(self.delay)
            GPIO.output(SM_STEP, GPIO.LOW)
            time.sleep(self.delay)
            if GPIO.input(SWITCH):
                trigger = False
                break
        # Move arm off of switch until it deactivates
        GPIO.output(SM_DIRECTION, GPIO.HIGH)
        while trigger2:
            GPIO.output(SM_STEP, GPIO.HIGH)
            time.sleep(self.delay)
            GPIO.output(SM_STEP, GPIO.LOW)
            time.sleep(self.delay)
            if not GPIO.input(SWITCH):
                time.sleep(3)
                trigger2 = False
                break

    def gripper(self, val, accel=5, speed=15):
        """
        :param val: True opens the gripper, False closes the gripper
        :param accel: acceleration of servo
        :param speed: speed of servo movement
        :return: void
        """
        self.servo.setAccel(self.gripper_pin, accel)  # set gripper acceleration
        self.servo.setSpeed(self.gripper_pin, speed)  # set gripper speed

        if val:
            self.servo.setTarget(self.gripper_pin, 3000)  # set gripper position
        else:
            self.servo.setTarget(self.gripper_pin, 8000)  # set gripper position

    def get_gripper_pos(self):
        """
        Getter for gripper position
        :return: Position of the gripper
        """
        return self.servo.getPosition(self.gripper_pin)  # get the current position of gripper servo

    def elbow(self, val, accel=5, speed=15):
        """
        :param val: True opens the gripper, False closes the gripper
        :param accel: acceleration of servo
        :param speed: speed of servo movement
        :return: void
        """
        self.servo.setAccel(self.elbow_pin, accel)  # set gripper acceleration
        self.servo.setSpeed(self.elbow_pin, speed)  # set gripper speed
        self.servo.setTarget(self.elbow_pin, val)  # set gripper position

    def get_elbow_pos(self):
        """
        Getter for gripper position
        :return: Position of the elbow
        """
        return self.servo.getPosition(self.elbow_pin)  # get the current position of elbow servo

    def pickup_can(self, msg):
        """
        Pickup the Can
        """
        self.picking_can = True

        while self.picking_can:
            print("Picking Up Can")
            # Open Gripper
            self.gripper(True)
            # Move Elbow down
            self.elbow(2000)
            rospy.sleep(4)

            # Close Gripper
            self.gripper(False)
            rospy.sleep(3)

            # Move Arm back up
            self.elbow(8500)
            rospy.sleep(3)

            # Open the Gripper
            self.gripper(True)
            rospy.sleep(3)

            # Done Picking up
            self.picking_can = False
            rospy.sleep(5)

        bool_msg = Bool()
        bool_msg.data = True
        self.pickup_pub.publish(bool_msg)

    def cleanup(self):
        """
        Cleans up if keyboard interrupt
        """
        self.servo.close()


if __name__ == "__main__":
    sc = ServoController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        sc.cleanup()
