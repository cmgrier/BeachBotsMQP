#!/usr/bin/env python

# Imports
import rospy
import pigpio
from geometry_msgs.msg import Point, Twist
from support.Constants import *


class Alignment:
    def __init__(self):
        """
        Initializations
        """

        # Initialization of the node
        rospy.init_node('Alignment')

        # Subscribers
        rospy.Subscriber('near_centroid', Point, self.centroid_callback)
        self.yaw_pub = rospy.Publisher('cam_yaw', Twist, queue_size=10)

        # Connect to local Pi.
        self.pi = pigpio.pi()

        # Configure the Camera Servo
        self.cam_servo_pin = SERVO_CAM
        self.position = 1000
        self.h = 480
        self.w = 500

        self.threshold = 40
        self.twitch = 70

    def centroid_callback(self, msg):
        """
        Callback for the centroid subscriber
        :param msg: Point
        :return: void
        """

        # Centroid from message
        centroid = (msg.x, msg.y)

        # Math for moving the camera
        video_centroid = (self.w / 2, self.h / 2 - 20)
        if centroid[0] > 0 and centroid[1] > 0:

            if centroid[1] > video_centroid[1] + self.threshold:
                self.position -= self.twitch
            elif centroid[1] < video_centroid[1] - self.threshold:
                self.position += self.twitch

        if 1000.0 > self.position > 500.0:
            self.pi.set_servo_pulsewidth(self.cam_servo_pin, self.position)
            rospy.sleep(0.5)

        self.yaw_alignment(centroid, video_centroid)

    def yaw_alignment(self, centroid, video_centroid, yaw_thresh=40):
        """
        Moves the motors until we are inline with the can
        :param centroid: the centroid tuple
        :param video_centroid: the video streams centroid
        :param yaw_thresh: the yaw threshold
        :return: void
        """
        turn_angle = 0

        if centroid[0] > 0 and centroid[1] > 0:
            if centroid[0] > video_centroid[0] + yaw_thresh:
                turn_angle = -1
            if centroid[0] < video_centroid[0] + yaw_thresh:
                turn_angle = 1

            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0

            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = turn_angle
        else:
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0

            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0

        self.yaw_pub.publish(msg)


if __name__ == "__main__":
    align_node = Alignment()
    rospy.spin()
