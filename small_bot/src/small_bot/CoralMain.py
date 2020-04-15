#!/usr/bin/env python3
# import the necessary packages
from imutils.video import VideoStream
from edgetpu.detection.engine import DetectionEngine
import imutils
from PIL import Image
import time
import pickle
import cv2
import socket
import argparse
import struct


class CoralMain:
    def __init__(self):
        """
        Initializations
        """

        print("[INFO] Beginning initialization of Coral Main File")

        # initialize the labels dictionary
        self.labels = {}
        self.labels[0] = "Can"

        # Argument Parser for model path
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

        # Socket Variables
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(('192.168.1.230', 8485))
        self.connection = self.client_socket.makefile('wb')
        self.img_counter = 0

        # Video Variables
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        self.h = 480  # DO NOT CHANGE
        self.w = 500  # DO NOT CHANGE
        self.threshold = 0.6

    def main_process(self):
        """
        Main Process
        :return: void
        """

        # loop over the frames from the video stream
        while True:
            # grab the frame from the threaded video stream and resize it
            frame = self.vs.read()

            # to have a maximum width of 500 pixels
            frame = imutils.resize(frame, width=500)
            orig = frame.copy()

            # prepare the frame for object detection by converting (1) it
            # from BGR to RGB channel ordering and then (2) from a NumPy
            # array to PIL image format
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # make predictions on the input frame
            frame = Image.fromarray(frame)
            results = self.model.DetectWithImage(frame, threshold=self.threshold, keep_aspect_ratio=True,
                                                 relative_coord=False)

            largest_box = (99, 99, -98, -98)
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
                    centroid = (int(startX + (endX - startX) / 2), int(startY + (endY - startY) / 2))
                    largest_box = (startX, startY, endX, endY)
                label = self.labels[r.label_id]
                # draw the bounding box and label on the image
                cv2.rectangle(orig, (startX, startY), (endX, endY),
                              (0, 255, 0), 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                text = "{}: {:.2f}%".format(label, r.score * 100)
                cv2.putText(orig, text, (startX, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Add the centroid of the image and the centroid of the largest object
            cv2.circle(orig, centroid, 4, (0, 255, 0), 2)
            cv2.circle(orig, (int(self.w / 2), int(self.h / 2) - 20), 4, (0, 0, 255), 2)

            # Socket Connection occurs here
            if centroid is None:
                centroid = (-99, -99)

            self.socket_con(orig, centroid, largest_box)

            key = cv2.waitKey(1) & 0xFF
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break

        # do a bit of cleanup
        cv2.destroyAllWindows()
        self.vs.stop()

    def socket_con(self, frame, centroid, largest_box):
        """
        Socket Connection
        :param frame: the image frame
        :param centroid: a tuple of the centroid
        :return: void
        """

        # Encode the image and pickle the frame and centroid into an array
        result, frame = cv2.imencode('.jpg', frame, self.encode_param)
        data = pickle.dumps((frame, centroid, largest_box), 0)
        size = len(data)

        # Send the image and data over the socket connection
        print("{}: {}".format(self.img_counter, size))
        self.client_socket.sendall(struct.pack(">L", size) + data)
        self.img_counter += 1


if __name__ == "__main__":
    cm = CoralMain()
    cm.main_process()
