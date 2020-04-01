#!/usr/bin/env python3

from edgetpu.detection.engine import DetectionEngine
from imutils.video import VideoStream
from PIL import Image
import imutils
import time
import cv2
import rospy
import numpy as np
import Path


class VisionScript:
    def __init__(self):
        """
        Initializations
        """

        self.threshold = 0.5

        self.model_path = Path("Google_Model/model.tflite")

        # load the Google Coral object detection model
        print("[INFO] loading Coral model...")
        self.model = DetectionEngine(self.model_path)

        # initialize the video stream and allow the camera sensor to warmup
        print("[INFO] starting video stream...")
        self.vs = VideoStream(src=0).start()

        time.sleep(2.0)
        print("Finished Initialization of Coral")

    def cam(self):
        """
        Camera Loop
        :return: Not sure yet
        """

        # loop over the frames from the video stream
        while True:
            # grab the frame from the threaded video stream and resize it
            # to have a maximum width of 500 pixels
            frame = self.vs.read()
            frame = imutils.resize(frame, width=500)
            # prepare the frame for object detection by converting (1) it
            # from BGR to RGB channel ordering and then (2) from a NumPy
            # array to PIL image format
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = Image.fromarray(frame)
            # make predictions on the input frame
            results = self.model.DetectWithImage(frame, threshold=self.threshold, keep_aspect_ratio=True,
                                                 relative_coord=False)

            key = cv2.waitKey(1) & 0xFF
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break
        # do a bit of cleanup
        cv2.destroyAllWindows()
        self.vs.stop()


