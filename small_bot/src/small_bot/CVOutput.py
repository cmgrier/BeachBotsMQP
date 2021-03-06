#!/usr/bin/env python

# Imports
import socket
import cv2
import maestro
import pickle
import rospy
import struct
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from support.Constants import *


class CVOutput:
    def __init__(self):
        """
        Initialization of the output from testing
        """

        # Initialization of the node
        rospy.init_node('CV_OUT')

        self.servo = maestro.Controller()

        # Configure the Camera Servo
        self.cam_servo_pin = CAMERA

        self.position = 4500  # DO NOT FORGET TO CHANGE IN ALIGNMENT AS WELL
        self.h = 480
        self.w = 500

        # Move Servo
        self.servo.setTarget(self.cam_servo_pin, self.position)
        rospy.sleep(0.5)

        # Publishers
        self.curr_pub = rospy.Publisher('curr_image_final', Image, queue_size=10)
        self.centroid_pub = rospy.Publisher('near_centroid', Point, queue_size=10)
        self.box_pub = rospy.Publisher('large_box', Point, queue_size=10)

        # Variable Declarations
        self.bridge = CvBridge()

        print("CV_OUTPUT Node ROS Finished Initializing")

        # Socket Connection
        self.HOST = ''
        self.PORT = 8485

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print('Socket created')

        self.s.bind((self.HOST, self.PORT))
        print('Socket bind complete')
        self.s.listen(10)
        print('Socket now listening')
        self.conn, self.addr = self.s.accept()

        self.data = b""
        self.payload_size = struct.calcsize(">L")
        print("payload_size: {}".format(self.payload_size))

        self.main_loop()

    def main_loop(self):
        """
        Main Loop
        :return: void
        """

        while True:

            while len(self.data) < self.payload_size:
                # print("Recv: {}".format(len(self.data)))
                self.data += self.conn.recv(4096)

            # print("Done Recv: {}".format(len(self.data)))
            packed_msg_size = self.data[:self.payload_size]  # Size of the full transfer
            self.data = self.data[self.payload_size:]  # Image data
            msg_size = struct.unpack(">L", packed_msg_size)[0]  # Size of message at beginning
            # print("msg_size: {}".format(msg_size))

            while len(self.data) < msg_size:
                self.data += self.conn.recv(4096)
            frame_and_centroid = self.data[:msg_size]
            self.data = self.data[msg_size:]

            (frame, centroid, area) = pickle.loads(frame_and_centroid)
            cent = (centroid[0], centroid[1])

            self.area_sender(area)
            self.centroid_sender(cent)

            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            self.curr_image_sender(frame)
            cv2.waitKey(1)

    def area_sender(self, area):
        """
        Largest Bounding box
        :param area: largest bounding box
        :return: void
        """

        msg = Point()
        msg.x = area
        msg.y = 0
        msg.z = 0

        self.box_pub.publish(msg)

    def curr_image_sender(self, frame):
        """
        Image Callback (Displays an image)
        :param frame: the image message
        :return: void
        """

        try:
            cv_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.curr_pub.publish(cv_image)
        except CvBridgeError as e:
            print(e)

    def centroid_sender(self, centroid):
        """
        Centroid Sender
        :param centroid: centroid as a tuple, non negative
        :return: void
        """

        # Message Decryption
        msg = Point()
        msg.x = centroid[0]
        msg.y = centroid[1]
        msg.z = 0

        # Message Publish
        self.centroid_pub.publish(msg)

    def cleanup(self):
        """
        Cleanup upon keyboard closure
        :return: void
        """
        self.servo.close()


if __name__ == "__main__":
    cv_out = CVOutput()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv_out.cleanup()
