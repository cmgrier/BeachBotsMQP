#!/usr/bin/env python

# Imports
import rospy
import socket
import cv2
import pickle
import struct
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class CVOutput:
    def __init__(self):
        """
        Initialization of the output from testing
        """

        # Initialization of the node
        rospy.init_node('CV_1')

        # Publishers
        self.curr_pub = rospy.Publisher('curr_image_final', Image, queue_size=10)

        # Variable Declarations
        self.bridge = CvBridge()

        print("CV_OUTPUT Node ROS Finished Initializing")

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
        :return:
        """

        while True:
            while len(self.data) < self.payload_size:
                print("Recv: {}".format(len(self.data)))
                self.data += self.conn.recv(4096)

            print("Done Recv: {}".format(len(self.data)))
            packed_msg_size = self.data[:self.payload_size]  # Size of the full transfer
            self.data = self.data[self.payload_size:]  # Image data
            msg_size = struct.unpack(">L", packed_msg_size)[0]  # Size of message at beginning
            print("msg_size: {}".format(msg_size))
            while len(self.data) < msg_size:
                self.data += self.conn.recv(4096)
            frame_data = self.data[14:msg_size]
            array_data = self.data[0:14]
            self.data = self.data[msg_size:]

            array = pickle.loads(array_data)
            rospy.loginfo("THIS IS THE ARRAY: ====")
            rospy.loginfo(array)

            frame = pickle.loads(frame_data)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            self.curr_image_sender(frame)
            cv2.waitKey(1)

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


if __name__ == "__main__":
    cv_out = CVOutput()
    rospy.spin()
