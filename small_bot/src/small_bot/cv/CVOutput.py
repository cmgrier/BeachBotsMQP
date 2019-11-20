#!/usr/bin/env python

# Imports
import numpy as np
import cv2
import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


class CVOutput:
    def __init__(self):
        """
        Initialization of the output from testing
        """

        # Subscribers
        rospy.Subscriber('init_image', Image, self.init_image_callback)
        rospy.Subscriber('curr_image', Image, self.curr_image_callback)

    def init_image_callback(self, msg):
        """
        Image Callback (Displays an image)
        :param msg: the image message
        :return: void
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg.data, "bgr8")
            cv2.imshow('initial image', cv_image)
        except CvBridgeError as e:
            print(e)

    def curr_image_callback(self, msg):
        """
        Image Callback (Displays an image)
        :param msg: the image message
        :return: void
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg.data, "bgr8")
            cv2.imshow('current image', cv_image)
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    cv_out = CVOutput()
    rospy.spin()
