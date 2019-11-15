#!/usr/bin/env python

import numpy as np
import cv2
import rospy
import sys
import roslib

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
#from small_bot.srv import ImageTransfer


class CVMain:
    def __init__(self):
        """
        Initializations for Computer Vision
        """
        # Initialization of Node
        rospy.init_node('CV')

        # Subscribers and Publishers
        rospy.Subscriber("/cv_trigger", Bool, self.is_running_callback)
        self.pub = rospy.Publisher("/blob_cords", Point)
        self.bridge = CvBridge()

        # Get the Camera Video
        self.cap = cv2.VideoCapture(0)

        # Initialization of variables
        self.isRunning = False

    def main_process(self):
        """
        Runs the main process on the video
        :return: void
        """
        while self.isRunning:

            # Capture frame-by-frame
            ret, frame = self.cap.read()

            # Initial Processing
            frame = self.brightness_and_contrast_auto(frame)

            # Main Video Processing
            frame = self.video_process(frame)

            self.get_cords(frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    @staticmethod
    def video_process(frame):

        return frame

    def brightness_and_contrast_auto(self, frame):



        return frame

    # def brightness_and_contrast_auto_service(self, frame):
    #
    #     rospy.wait_for_service('brightness_and_contrast')
    #     try:
    #         try:
    #             cv_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
    #             service_request = rospy.ServiceProxy('brightness_and_contrast', ImageTransfer)
    #             new_frame = service_request(cv_image)
    #             return new_frame
    #
    #         except CvBridgeError as e:
    #             print(e)
    #
    #     except rospy.ServiceException as e:
    #
    #         print("Service call failed: %s" % e)

    def get_cords(self, frame):

        # Make new point
        new_point = Point()

        # Make Point message
        new_point.x = 0
        new_point.y = 0
        new_point.z = 0

        # Publish point
        self.pub.publish(new_point)

    def is_running_callback(self, msg):
        """
        Callback for running
        :param msg: the Boolean msg
        :return: void
        """
        self.isRunning = msg.data
        if self.isRunning:
            self.main_process()


if __name__ == "__main__":
    cv_main = CVMain()
