#!/usr/bin/env python

import rospy
from support.PID import PID
from support.Constants import *

class Navigate:
    def __init__(self):
        self.PID = None
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber("encoder", Encoder, self.positionListener())
        rospy.Subscriber("imu", IMU, self.positionListener())
        self.position = 0
        self.angle = 0

    def positionListener(self, data):
        """
        Callback for the encoder topic
        :param data: Linear position based on encoder ticks
        :return:
        """
        self.position = data.encoderDistance

    def angleListener(self, data):
        """
        Callback for the imu topic
        :param data: Angular position in degrees based on IMU readings
        :return:
        """
        self.angle = data.imuAngle

    def withinDistanceThreshold(self, dist):
        """
        Determines if the current position meets the threshold for the desired position
        :param dist: Linear distance in inches
        :return: A boolean
        """
        if self.position >= dist - DISTANCE_THREHOLD_MIN and self.position <= dist + DISTANCE_THRESHOLD_MAX:
            return True
        return False

    def withinAngleThreshold(self, angle):
        """
        Determines if the current angle meets the threshold for the desired angle
        :param angle:
        :return: A boolean
        """
        if self.angle >= angle - ANGLE_THREHOLD_MIN and self.angle <= angle + ANGLE_THRESHOLD_MAX:
            return True
        return False

    def drive_distance(self, dist):
        """
        Drive a set distance forward
        :param dist: The desired linear distance
        :return:
        """
        self.PID = PID(DRIVE_DIST_KP, DRIVE_DIST_IP, DRIVE_DIST_DP)
        while self.withinDistanceThreshold(dist):
            rospy.wait_for_message("encoder")
            #TODO add the beef

    def turn_angle(self, angle):
        """
        Turn to the desired angle
        :param angle: The desired angle in degrees
        :return:
        """
        self.PID = PID(TURN_ANGLE_KP, TURN_ANGLE_IP, TURN_ANGLE_DP)
        while self.withinAngleThreshold(angle):
            rospy.wait_for_message("imu")
            #TODO add the beef
