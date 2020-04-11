#!/usr/bin/env python3
# import the necessary packages
from imutils.video import VideoStream
from edgetpu.detection.engine import DetectionEngine
import RPi.GPIO as GPIO
import imutils
from PIL import Image
import time
import pickle
import cv2
import socket
import argparse
import struct
from support.Constants import *


class CoralMain:
    def __init__(self):
        """
        Initializations
        """

        # Configure the Camera Servo
        self.cam_servo_pin = SERVO_CAM
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.cam_servo_pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.cam_servo_pin, 50)

        self.position = 3
        # Move Servo
        self.servo.start(self.position)  # Start
        time.sleep(.5)  # Wait
        # self.servo.stop()  # Stop

        # Initialization of random variables
        self.threshold = 0.5

        # initialize the labels dictionary
        self.labels = {}
        self.labels[0] = "Can"

        ap = argparse.ArgumentParser()
        ap.add_argument("-m", "--model", required=True,
                        help="path to TensorFlow Lite object detection model")

        args = vars(ap.parse_args())

        # load the Google Coral object detection model
        print("[INFO] loading Coral model...")
        self.model = DetectionEngine(args["model"])

        # initialize the video stream and allow the camera sensor to warmup
        self.vs = VideoStream(src=0).start()

        time.sleep(2.0)

        print("Finished Initialization of Model and Video")

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(('192.168.1.230', 8485))
        self.connection = self.client_socket.makefile('wb')

        self.img_counter = 0

        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        self.h = None
        self.w = 500  # DO NOT CHANGE

    def main_process(self):
        """
        Main Process
        :return: void
        """

        # loop over the frames from the video stream
        while True:
            # grab the frame from the threaded video stream and resize it
            # to have a maximum width of 500 pixels
            frame = self.vs.read()
            if self.h is None:
                self.h, w, c = frame.shape
            frame = imutils.resize(frame, width=500)
            orig = frame.copy()
            # prepare the frame for object detection by converting (1) it
            # from BGR to RGB channel ordering and then (2) from a NumPy
            # array to PIL image format
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # make predictions on the input frame
            frame = Image.fromarray(frame)
            results = self.model.DetectWithImage(frame, threshold=self.threshold, keep_aspect_ratio=True, relative_coord=False)

            largest_area = -999
            centroid = None
            # loop over the results
            for r in results:
                # extract the bounding box and box and predicted class label
                box = r.bounding_box.flatten().astype("int")
                (startX, startY, endX, endY) = box
                area = (endY - startY) * (endX - startX)
                if area > largest_area:
                    largest_area = area
                    centroid = (startX + (endX - startX)/2, startY - (endY - startY)/2)
                label = self.labels[r.label_id]
                # draw the bounding box and label on the image
                cv2.rectangle(orig, (startX, startY), (endX, endY),
                              (0, 255, 0), 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                text = "{}: {:.2f}%".format(label, r.score * 100)
                cv2.putText(orig, text, (startX, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if centroid is not None:
                self.go_to(centroid)

            # Socket Connection occurs here
            self.socket_con(orig)

            key = cv2.waitKey(1) & 0xFF
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break
        # do a bit of cleanup
        cv2.destroyAllWindows()
        self.vs.stop()
        self.servo.stop()

    def go_to(self, centroid, threshold=50, twitch=0.2):
        """
        Sends the servo to a given position
        :param centroid: tuple of coordinates
        :param threshold: threshold
        :param twitch: how much the servo moves.
        :return: void
        """

        video_centroid = (self.w/2, self.h/2)
        if centroid[1] > video_centroid[1]:
            self.position -= twitch
        if centroid[1] < video_centroid[1]:
            self.position += twitch
        if 100.0 > self.position > 0.0:
            self.servo.ChangeDutyCycle(self.position)
            print(self.position)
            time.sleep(.05)

    def socket_con(self, frame):
        """
        Socket Connection
        :param frame:
        :return:
        """
        result, frame = cv2.imencode('.jpg', frame, self.encode_param)
        data = pickle.dumps(frame, 0)
        size = len(data)

        print("{}: {}".format(self.img_counter, size))
        self.client_socket.sendall(struct.pack(">L", size) + data)
        self.img_counter += 1


if __name__ == "__main__":
    cm = CoralMain()
    cm.main_process()
