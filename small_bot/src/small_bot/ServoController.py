#!/usr/bin/env python
import maestro
import rospy
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
        # Open the Gripper
        self.gripper(True)
        rospy.sleep(3)
        self.gripper(False)
        # Move the joint to over the bucket
        self.elbow(9000)
        # rospy.sleep(3)
        self.elbow(3000)

    def gripper(self, val, accel=4, speed=10):
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

    def elbow(self, val, accel=4, speed=10):
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

    def pickup_can(self):
        """
        Pickup the Can
        """
        self.picking_can = True

        while self.picking_can:
            print("Picking Up Can")
            # Open Gripper
            self.gripper(True)
            # Move Elbow down
            self.elbow(1000)
            rospy.sleep(4)

            # Close Gripper
            self.gripper(False)
            rospy.sleep(3)

            # Move Arm back up
            self.elbow(1000)
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
