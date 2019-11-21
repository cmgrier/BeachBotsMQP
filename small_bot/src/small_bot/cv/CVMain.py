#!/usr/bin/env python

# Imports
import numpy as np
import cv2
import rospy
import time
import sys
import RPi.GPIO as GPIO
from support.Constants import *
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


class CVMain:
    def __init__(self):
        """
        Initializations for Computer Vision
        """
        # Initialization of Node
        rospy.init_node('CV')

        # Configure the Camera Servo
        self.cam_servo_pin = SERVO_CAM
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.cam_servo_pin, GPIO.OUT)

        self.servo = GPIO.PWM(self.cam_servo_pin, 50)
        self.servo.start(10)

        # Subscribers and Publishers
        rospy.Subscriber("cv_trigger", Bool, self.is_running_callback)
        self.pub = rospy.Publisher("blob_cords", Point, queue_size=1)

        self.init_image_pub = rospy.Publisher("init_image", CompressedImage, queue_size=1)
        self.curr_image_pub = rospy.Publisher("curr_image", CompressedImage, queue_size=1)

        # Initialization of variables
        self.bridge = CvBridge()
        self.isRunning = False

        print("Finished Initialization of CV")

    def main_process(self):
        """
        Runs the main process on the video
        :return: void
        """

        cap = cv2.VideoCapture(0)

        while self.isRunning:

            # Image Acquisition
            ret, frame = cap.read()

            # Publish the original image (MOVE THIS TO TEST FUNCTIONS)
            self.init_image_pub.publish(self.make_compressed_msg(frame))

            # Image Enhancements
            frame = self.enhancement(frame)

            # Publish the fixed Image (MOVE THIS STATEMENT TO TEST FUNCTIONS)
            self.curr_image_pub.publish(self.make_compressed_msg(frame))

            # Segmentation
            frame = self.segmentation(frame)

            # Post Processing
            frame = self.post_processing(frame)

            # Information Extraction
            x, y = self.info_extract(frame)

            # Current Handler for no cords
            if x < 10000:
                # Publish Information
                self.pub_cords(x, y)

            time.sleep(.2)

            # Necessary to make loop run
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    @staticmethod
    def make_compressed_msg(frame):
        """
        Make a compressed msg
        :param frame: a uncompressed image
        :return: a compressed image
        """

        # Make a compressed image
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()

        # Return the compressed image
        return msg

    @staticmethod
    def enhancement(frame):
        """
        NOTE: COULD USE MORE ENHANCEMENT FOR NOW IT LOOKS FINE TO ME
        Computer Vision Image Enhancement function
        :param frame: a frame
        :return: a modified frame
        """

        # Convert the image into YUV
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)

        # Equalize the histogram of the Y channel
        frame[:, :, 0] = cv2.equalizeHist(frame[:, :, 0])

        # Convert the image back to RGB
        return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR)

    @staticmethod
    def segmentation(frame):
        """
        Computer Vision Image Segmentation where we will distinguish unusual sand things
        :param frame: a frame
        :return: a modified frame
        """

        # Convert to Gray scale

        # threshold them

        return frame

    @staticmethod
    def post_processing(frame):
        """
        Computer Vision Post Processing (Fixes Segmentation)
        :param frame: a frame
        :return: a modified frame
        """

        # Get rid of any static

        # Improve Finally

        return frame

    @staticmethod
    def info_extract(frame):
        """
        Extracts the information from a given frame
        :param frame: a frame
        :return: a tuple of x and y from bottom middle
        """

        # Extract the nearest largest object

        # If not return x larger than a 10000 so that it knows its wrong

        x = 0
        y = 0

        return x, y

    def pub_cords(self, x, y):
        """
        Publish a tuple as a point
        :param x: the x distance from center, + is right, - is left
        :param y: the y distance from bottom, only +
        :return: void
        """

        # Make new point
        new_point = Point()

        # Make Point message
        new_point.x = x
        new_point.y = y
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
        print(str(msg.data))
        if self.isRunning:
            self.main_process()


if __name__ == "__main__":
    cv_main = CVMain()
    rospy.spin()
