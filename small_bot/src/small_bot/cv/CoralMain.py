#!/usr/bin/env python
# import the necessary packages
from imutils.video import VideoStream
import imutils
import time
import cv2
import rospy
import numpy as np
from support.Constants import *
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from ModelScript import ModelScript


class CoralMain:
    def __init__(self):
        """
        Initializations
        """
        rospy.loginfo("[INFO] Initializing Class and Node")

        # Initialization of Node
        rospy.init_node('Coral')

        # Configure the Camera Servo
        self.cam_servo_pin = SERVO_CAM
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.cam_servo_pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.cam_servo_pin, 50)

        # Move Servo
        self.servo.start(5)  # Start
        time.sleep(.5)  # Wait
        self.servo.stop()  # Stop

        # Subscribers and Publishers
        # rospy.Subscriber("cv_trigger", Bool, self.is_running_callback)
        self.curr_image_pub = rospy.Publisher("curr_image", CompressedImage, queue_size=1)

        # Initialization of random variables
        self.bridge = CvBridge()
        self.isRunning = False
        self.threshold = 0.5

        # initialize the labels dictionary
        rospy.loginfo("[INFO] parsing class labels...")
        self.labels = {}
        self.labels[0] = "Can"

        # initialize the video stream and allow the camera sensor to warmup
        print("[INFO] starting video stream...")
        self.vs = VideoStream(src=0).start()

        time.sleep(2.0)
        self.model_script = ModelScript()

        print("Finished Initialization of Node")

        self.main_process()

    def main_process(self):
        """
        Main Process
        :return: void
        """
        rospy.loginfo("In Main Process")

        # loop over the frames from the video stream
        while True:
            # grab the frame from the threaded video stream and resize it
            # to have a maximum width of 500 pixels
            frame = self.vs.read()
            frame = imutils.resize(frame, width=500)
            orig = frame.copy()
            # prepare the frame for object detection by converting (1) it
            # from BGR to RGB channel ordering and then (2) from a NumPy
            # array to PIL image format
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # make predictions on the input frame
            frame = self.model_script.image(frame)
            results = self.model_script.detect(frame, self.threshold)

            # loop over the results
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
            # show the output frame and wait for a key press

            self.curr_image_pub.publish(self.make_compressed_msg(orig))
            # cv2.imshow("Frame", orig)

            key = cv2.waitKey(1) & 0xFF
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break
        # do a bit of cleanup
        cv2.destroyAllWindows()
        self.vs.stop()

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
