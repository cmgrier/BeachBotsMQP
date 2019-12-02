#!/usr/bin/env python

# Imports
import numpy as np
import cv2
import rospy
import time
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

        self.servo.start(5)  # Start
        time.sleep(.5)  # Wait
        self.servo.stop()  # Stop

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

            # Image Enhancements
            frame = self.enhancement(frame)

            # Publish the original image (MOVE THIS TO TEST FUNCTIONS)
            self.init_image_pub.publish(self.make_compressed_msg(frame))

            # Segmentation
            frame = self.segmentation(frame)

            # Publish the fixed Image (MOVE THIS STATEMENT TO TEST FUNCTIONS)
            # self.curr_image_pub.publish(self.make_compressed_msg(frame))

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
        Computer Vision Image Enhancement function
        :param frame: a frame
        :return: a modified frame
        """

        # Convert the image into YUV
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)

        # Equalize the histogram of the Y channel
        frame[:, :, 0] = cv2.equalizeHist(frame[:, :, 0])

        # Convert the image back to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR)

        # Blur the image and return the image (Possibly insert crop for the sky)
        return frame  # cv2.blur(frame, (5, 5))

    def segmentation(self, frame):
        """
        Computer Vision Image Segmentation where we will distinguish unusual sand things
        :param frame: a frame
        :return: a modified frame
        """
        # Get the size of the image
        height, width, channels = frame.shape
        buffer = 10

        # Calculate y1, y2, x1, x2 for small segment in bottom left
        l_y1 = height / 2 + buffer
        l_y2 = height - buffer

        l_x1 = 0 + buffer
        l_x2 = width / 2 + buffer

        # Calculate y1, y2, x1, x2 for small segment in bottom right
        r_y1 = height / 2 + buffer
        r_y2 = height - buffer

        r_x1 = width / 2 + buffer
        r_x2 = width - buffer

        # Get filters from a small left corner
        left_low_filter, left_high_filter = self.small_segment_filter_generator(frame, l_y1, l_y2, l_x1, l_x2)
        # print(left_low_filter)
        # print(left_high_filter)

        # Get filters from the small right corner
        right_low_filter, right_high_filter = self.small_segment_filter_generator(frame, r_y1, r_y2, r_x1, r_x2)

        # Use the right to check the left
        # TODO Implement a check here

        # filter the image
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # new_image = cv2.inRange(frame, left_low_filter, left_high_filter)

        return frame

    @staticmethod
    def small_segment_filter_generator(frame, y1, y2, x1, x2, expansion=20):
        """
        Uses coordinates and a frame to generate the lower and higher filters for a small segment of the frame
        :param frame: a frame
        :param y1: top left y cord
        :param y2: bottom right y cord
        :param x1: top left x cord
        :param x2: bottom right x cord
        :param expansion: expansion factor
        :return: a lower and higher expansion tuple
        """

        # Get small left corner of an image
        small_seg = frame[y1:y2, x1:x2]

        # Get the hsv of that image
        hsv_frame = cv2.cvtColor(small_seg, cv2.COLOR_BGR2HSV)

        # Get the different channels histograms
        hue_hist = cv2.calcHist(hsv_frame, [0], None, [256], [0, 256])
        sat_hist = cv2.calcHist(hsv_frame, [1], None, [256], [0, 256])
        value_hist = cv2.calcHist(hsv_frame, [2], None, [256], [0, 256])

        # Determine the high points
        # max_hue = np.amax(hue_hist)
        hue_max_index = np.where(hue_hist == np.amax(hue_hist))
        # print(hue_max_index[0])
        hue_index = int(hue_max_index[0][0])

        # max_sat = np.amax(sat_hist)
        sat_max_index = np.where(sat_hist == np.amax(sat_hist))
        # print(sat_max_index[0])
        sat_index = int(sat_max_index[0][0])

        # max_value = np.amax(value_hist)
        value_max_index = np.where(value_hist == np.amax(value_hist))
        # print(value_max_index[0])
        value_index = int(value_max_index[0][0])

        # Expansion of each low index
        if hue_index < expansion:
            low_hue = 0
        else:
            low_hue = hue_index - expansion

        if sat_index < expansion:
            low_sat = 0
        else:
            low_sat = sat_index - expansion

        if value_index < expansion:
            low_value = 0
        else:
            low_value = value_index - expansion

        hist_length = len(hue_hist)

        # Expansion of each high index
        if hue_index > hist_length - expansion:
            high_hue = hist_length
        else:
            high_hue = hue_index + expansion

        if sat_index > hist_length - expansion:
            high_sat = hist_length
        else:
            high_sat = sat_index + expansion

        if value_index > hist_length - expansion:
            high_value = hist_length
        else:
            high_value = value_index + expansion

        # Put together the filter
        low_filter = np.array([low_hue, low_sat, low_value])
        high_filter = np.array([high_hue, high_sat, high_value])

        return low_filter, high_filter

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
