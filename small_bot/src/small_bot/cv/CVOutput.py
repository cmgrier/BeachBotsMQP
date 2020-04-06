#!/usr/bin/env python

# Imports
import numpy as np
import cv2
import rospy
import sys
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


class CVOutput:
    def __init__(self):
        """
        Initialization of the output from testing
        """

        # Initialization of the node
        rospy.init_node('CV_1')

        # Subscribers
        rospy.Subscriber('init_image', CompressedImage, self.init_image_callback)
        rospy.Subscriber('curr_image', CompressedImage, self.curr_image_callback)

        # Publishers
        self.init_pub = rospy.Publisher('init_image_final', Image, queue_size=10)
        self.curr_pub = rospy.Publisher('curr_image_final', Image, queue_size=10)

        # Variable Declarations
        self.bridge = CvBridge()

        print("CV_1 Node Initialized")

    def init_image_callback(self, msg):
        """
        Image Callback (Displays an image)
        :param msg: the image message
        :return: void
        """

        # Image to numpy array
        nparr = np.fromstring(msg.data, np.uint8)
        img_decode = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        try:
            cv_image = self.bridge.cv2_to_imgmsg(img_decode, "bgr8")
            self.init_pub.publish(cv_image)
        except CvBridgeError as e:
            print(e)

    def curr_image_callback(self, msg):
        """
        Image Callback (Displays an image)
        :param msg: the image message
        :return: void
        """
        # Image to numpy array
        nparr = np.fromstring(msg.data, np.uint8)
        img_decode = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        try:
            cv_image = self.bridge.cv2_to_imgmsg(img_decode, "bgr8")
            self.curr_pub.publish(cv_image)
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    cv_out = CVOutput()
    rospy.spin()
