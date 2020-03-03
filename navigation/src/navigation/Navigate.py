#!/usr/bin/env python
# title           :Navigate.py
# description     :drive smallbot to a specific coordinate
# author          :Sean Tidd
# date            :2020-02-11
# version         :0.1
# notes           :
# python_version  :3.5
# ==============================================================================
import rospy
from support.PID import PID
from support.Constants import *
from geometry_msgs.msg import Pose, Twist
import math


class Navigate:
    def __init__(self):
        self.PID = None
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber("odom", Pose, self.position_listener)
        self.pub = rospy.Publisher("cmd_vel",Twist, queue_size = 10 )
        self.position = (0.0,0.0)
        self.old_position = (0.0,0.0)
        self.angle = 0.0
        self.old_angle = 0.0


    def position_listener(self, data):
        """
        Callback for the encoder topic
        :param data: Pose message
        :return:
        """
        self.position = (data.position.x, data.position.y)
        self.angle = data.orientation.z
        print(self.position, " : ", self.angle)


    def get_dist(self, x, y, x2, y2):
        """
        Gets the distance between two points
        :param x: first x-coord in meters
        :param y: first y-coord in meters
        :param x2: second x-coord in meters
        :param y2: second y-coord in meters
        :return: distance formula
        """
        return math.sqrt(((x-x2)*(x-x2)) + ((y-y2)*(y-y2)))


    def get_angle(self, x, y, x2, y2):
        """
        Gets the angle between two points
        :param x: first x-coord in meters
        :param y: first y-coord in meters
        :param x2: second x-coord in meters
        :param y2: second y-coord in meters
        :return: angle formula
        """
        return (180/math.pi)*math.atan((y2-y)/(x2-x))


#WORKS MAY CHAMGE
    def within_distance_threshold(self, dist):
        """
        Determines if the current position meets the threshold for the desired position
        :param dist: Linear distance in meters
        :return: True if current distance is within desired distance threshold
        """
        active_dist = self.get_dist(self.old_position[0],self.old_position[1],self.position[0],self.position[1])
        print(active_dist, " : ", dist)
        if active_dist >= dist - DISTANCE_THRESHOLD_MIN and active_dist <= dist + DISTANCE_THRESHOLD_MAX:
            return True
        return False

#WORKS MAY CHAMGE
    def within_angle_threshold(self, angle):
        """
        Determines if the current angle meets the threshold for the desired angle
        :param angle:
        :return: A boolean
        """
        activeAngle = self.angle - self.old_angle
        print(activeAngle, " : ", angle)
        if activeAngle >= angle - ANGLE_THRESHOLD_MIN and activeAngle <= angle + ANGLE_THRESHOLD_MAX:
            return True
        return False

#
    def set_speed_limits(self,speed):
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

        while not self.within_distance_threshold(dist) and not rospy.is_shutdown():
            active_dist = self.get_dist(self.old_position[0],self.old_position[1],self.position[0],self.position[1])
            distPID.update(active_dist)
            msg.linear.x = self.set_speed_limits(distPID.output/10.0)
            msg.angular.z = 0.0
            self.pub.publish(msg)
            print("Dist: ",dist, "| active_dist: ", active_dist, " | PID Output: ",distPID.output)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    #
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
        while not self.within_angle_threshold(angle) and not rospy.is_shutdown():
            activeAng = self.angle - self.old_angle
            angPID.update(activeAng)
            msg.linear.x = 0.0
            msg.angular.z = self.set_speed_limits(angPID.output / 10.0)
            self.pub.publish(msg)
            print("Amg: ", angle, "| activeAmg: ", activeAng, " | PID Output: ", angPID.output)
            r.sleep()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)


    def drive_to_coord(self, x, y):
        """
        Makes the rovot drive to the specified coordinate
        :param x: Desired x-coord in meters
        :param y: Desired y-coord in meters
        :return: True when finished executing
        """
        self.old_position = self.position
        self.old_angle = self.angle
        distTarget = self.get_dist(self.position[0],self.position[1],x,y)
        angleTarget = self.get_angle(self.position[0],self.position[1],x,y)
        self.turn_angle(angleTarget)
        self.drive_distance(distTarget)
        return True

if __name__=="__main__":
    nav = Navigate()
    nav.turn_angle(90)
