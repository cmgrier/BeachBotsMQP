#!/usr/bin/env python

import numpy as np
import cv2
import rospy
import time
import RPi.GPIO as GPIO
from VisionScript import VisionScript as vis
from support.Constants import *
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from imutils.video import VideoStream


class CVNode:
    def __init__(self):
        """
        Initialization
        """

        # Initialization of Node
        rospy.init_node('CVNode')

        # Configure the Camera Servo
        self.cam_servo_pin = SERVO_CAM
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.cam_servo_pin, GPIO.OUT)

        self.servo = GPIO.PWM(self.cam_servo_pin, 50)

        self.servo.start(5)  # Start
        time.sleep(.5)  # Wait
        self.servo.stop()  # Stop

        self.curr_image_pub = rospy.Publisher("curr_image", CompressedImage, queue_size=1)

        rospy.loginfo("[INFO] parsing class labels...")
        self.labels = {}
        self.labels[0] = "Can"

        self.camera_node = vis()

    def process_and_send(self, results):
        """
        Processes and sends to CVOutput
        :return:
        """
        # loop over the results
        print("[INFO] starting video stream...")
        vs = VideoStream(src=0).start()

        orig = vs.read()

        vs.stop()

        for r in results:
            # extract the bounding box and box and predicted class label
            box = r.bounding_box.flatten().astype("int")
            (startX, startY, endX, endY) = box
            label = self.labels[r.label_id]
            # draw the bounding box and label on the image
            cv2.rectangle(orig, (startX, startY), (endX, endY),
                          (0, 255, 0), 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            text = "{}: {:.2f}%".format(label, r.score * 100)
            cv2.putText(orig, text, (startX, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        self.curr_image_pub.publish(self.make_compressed_msg(orig))

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
