#!/usr/bin/env python

import rospy
from support.PID import PID
from support.Constants import *
from geometry_msgs.msg import Pose, Twist
import math


class Navigate:
    def __init__(self):
        self.PID = None
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber("odom", Pose, self.positionListener)
        self.pub = rospy.Publisher("cmd_vel",Twist, queue_size = 10 )
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
        self.position = (data.position.x, data.position.y)
        self.angle = data.orientation.z
        print(self.position, " : ", self.angle)


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
        return (180/math.pi)*math.atan((y2-y)/(x2-x))



    def withinDistanceThreshold(self, dist):
        """
        Determines if the current position meets the threshold for the desired position
        :param dist: Linear distance in meters
        :return: True if current distance is within desired distance threshold
        """
        activeDist = self.getDist(self.oldPosition[0],self.oldPosition[1],self.position[0],self.position[1])
        print(activeDist, " : ", dist)
        if activeDist >= dist - DISTANCE_THRESHOLD_MIN and activeDist <= dist + DISTANCE_THRESHOLD_MAX:
            return True
        return False


    def withinAngleThreshold(self, angle):
        """
        Determines if the current angle meets the threshold for the desired angle
        :param angle:
        :return: A boolean
        """
        activeAngle = self.angle - self.oldAngle
        print(activeAngle, " : ", angle)
        if activeAngle >= angle - ANGLE_THRESHOLD_MIN and activeAngle <= angle + ANGLE_THRESHOLD_MAX:
            return True
        return False


    def setSpeedLimits(self,speed):
        if speed > 1.00:
            return 1.00
        elif speed < -1.00:
            return -1.00
        else:
            return speed



    def drive_distance(self, dist):
        """
        Drive a set distance forward
        :param dist: The desired linear distance
        :return:
        """
        distPID = PID(DRIVE_DIST_KP, DRIVE_DIST_IP, DRIVE_DIST_DP)
        distPID.SetPoint = dist
        msg = Twist()

        while not self.withinDistanceThreshold(dist) and not rospy.is_shutdown():
            activeDist = self.getDist(self.oldPosition[0],self.oldPosition[1],self.position[0],self.position[1])
            distPID.update(activeDist)
            msg.linear.x = self.setSpeedLimits(distPID.output/10.0)
            msg.angular.z = 0.0
            self.pub.publish(msg)
            print("Dist: ",dist, "| activeDist: ", activeDist, " | PID Output: ",distPID.output)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    
    def turn_angle(self, angle):
        """
        Turn to the desired angle
        :param angle: The desired angle in degrees
        :return:
        """
        angPID = PID(TURN_ANGLE_KP, TURN_ANGLE_IP, TURN_ANGLE_DP)
        angPID.SetPoint = angle
        msg = Twist()
        r = rospy.Rate(10)
        while not self.withinAngleThreshold(angle) and not rospy.is_shutdown():
            activeAng = self.angle - self.oldAngle
            angPID.update(activeAng)
            msg.linear.x = 0.0
            msg.angular.z = self.setSpeedLimits(angPID.output / 10.0)
            self.pub.publish(msg)
            print("Amg: ", angle, "| activeAmg: ", activeAng, " | PID Output: ", angPID.output)
            r.sleep()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    #
    def driveToCoord(self, x, y):
        """
        Makes the rovot drive to the specified coordinate
        :param x: Desired x-coord in meters
        :param y: Desired y-coord in meters
        :return: True when finished executing
        """
        self.oldPosition = self.position
        self.oldAngle = self.angle
        distTarget = self.getDist(self.position[0],self.position[1],x,y)
        angleTarget = self.getAngle(self.position[0],self.position[1],x,y)
        self.turn_angle(angleTarget)
        self.drive_distance(distTarget)
        return True

if __name__=="__main__":
    nav = Navigate()
    print(nav.getDist(1,1,4,7))
    print(nav.getAngle(1,1,4,7))
    dist = nav.getDist(1, 1, 4, 7)
    nav.oldPosition = nav.position
    nav.turn_angle(90)
