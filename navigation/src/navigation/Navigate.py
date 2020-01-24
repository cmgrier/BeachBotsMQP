#!/usr/bin/env python

import rospy
from support.PID import PID
from support.Constants import *
from geometry_msgs.msg import Pose
import math

class Navigate:
    def __init__(self):
        self.PID = None
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber("odom", Pose, self.positionListener())
        self.position = (0.0,0.0)
        self.oldPosition = (0.0,0.0)
        self.angle = 0.0
        self.oldAngle = 0.0

    def positionListener(self, data):
        """
        Callback for the encoder topic
        :param data: Pose message
        :return:
        """
        self.position = (data.linear.x, data.linear.y)
        self.angle = data.orientation.z

    def getDist(self, x, y, x2, y2):
        """
        Gets the distance vetween two points
        :param x: first x-coord in meters
        :param y: first y-coord in meters
        :param x2: second x-coord in meters
        :param y2: second y-coord in meters
        :return: distance formula
        """
        return math.sqrt(((x-x2)*(x-x2)) + ((y-y2)*(y-y2)))

    def getAngle(self, x, y, x2, y2):
        """
        Gets the angle vetween two points
        :param x: first x-coord in meters
        :param y: first y-coord in meters
        :param x2: second x-coord in meters
        :param y2: second y-coord in meters
        :return: angle formula
        """
        return math.atan((y2-y)/(x2-x))

    def withinCoordinatesThreshold(self, dist):
        """
        Determines if the current position meets the threshold for the desired position
        :param dist: Linear distance in meters
        :return: True if current distance is within desired distance threshold
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

    def driveToCoord(self, x, y):
        """
        Makes the rovot drive to the specified coordinate
        :param x: Desired x-coord in meters
        :param y: Desired y-coord in meters
        :return: True when finished executing
        """
        #TODO

if __name__=="__main__":
    nav = Navigate()
    print(nav.getDist(1,1,4,7))
    print(nav.getAngle(1,1,4,7))