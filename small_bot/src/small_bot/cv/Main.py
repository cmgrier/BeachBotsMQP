# This will be the main for the Computer Vision

import numpy as np
import cv2


class Main:

    def __init__(self):

        self.cap = cv2.VideoCapture(0)
        self.test_cam()

    def test_cam(self):
        while True:
            # Capture frame-by-frame
            ret, frame = self.cap.read()

            # Our operations on the frame come here
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Display the resulting frame
            cv2.imshow('frame', gray)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


if __name__ == "__main__":
    m = Main()
