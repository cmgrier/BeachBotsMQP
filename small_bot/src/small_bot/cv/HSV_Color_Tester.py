#!/usr/bin/env
"""
HSV Color Testing File
Created by: Everett Johnson
Developed on: 7/12/19

Use: Used to determine colors for connectors on different lines

NOTE: This had to be made its own file so video feed could be implemented
"""

# Imports
from __future__ import print_function
import cv2
import argparse


class HSVColorTest:
    def __init__(self):
        """
        Initializes the Tester
        """

        # Initialized Values
        self.max_value = 255
        self.max_value_H = 360 // 2
        self.low_H = 0
        self.low_S = 0
        self.low_V = 0
        self.high_H = self.max_value_H
        self.high_S = self.max_value
        self.high_V = self.max_value
        self.window_capture_name = 'Video Capture'
        self.window_detection_name = 'Object Detection'
        self.low_H_name = 'Low H'
        self.low_S_name = 'Low S'
        self.low_V_name = 'Low V'
        self.high_H_name = 'High H'
        self.high_S_name = 'High S'
        self.high_V_name = 'High V'

        # Parser Variables
        self.parser = argparse.ArgumentParser(description='Code for Threshold Operations using inRange tutorial.')
        self.parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
        self.args = self.parser.parse_args()
        self.cap = cv2.VideoCapture(self.args.camera)

        # Window Customization
        cv2.namedWindow(self.window_capture_name)
        cv2.namedWindow(self.window_detection_name)

        cv2.createTrackbar(self.low_H_name, self.window_detection_name, self.low_H, self.max_value_H,
                           self.on_low_h_thresh_track_bar)
        cv2.createTrackbar(self.high_H_name, self.window_detection_name, self.high_H, self.max_value_H,
                           self.on_high_h_thresh_track_bar)
        cv2.createTrackbar(self.low_S_name, self.window_detection_name, self.low_S, self.max_value,
                           self.on_low_s_thresh_track_bar)
        cv2.createTrackbar(self.high_S_name, self.window_detection_name, self.high_S, self.max_value,
                           self.on_high_s_thresh_track_bar)
        cv2.createTrackbar(self.low_V_name, self.window_detection_name, self.low_V, self.max_value,
                           self.on_low_v_thresh_track_bar)
        cv2.createTrackbar(self.high_V_name, self.window_detection_name, self.high_V, self.max_value,
                           self.on_high_v_thresh_track_bar)

        self.update()

    def update(self):
        """
        This Function updates the video feed as slider values are changed
        :return: void
        """
        while True:
            ret, frame = self.cap.read()
            if frame is None:
                break
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            frame_threshold = cv2.inRange(frame_hsv, (self.low_H, self.low_S, self.low_V),
                                          (self.high_H, self.high_S, self.high_V))

            cv2.imshow(self.window_capture_name, frame)
            cv2.imshow(self.window_detection_name, frame_threshold)

            key = cv2.waitKey(30)
            if key == ord('q') or key == 27:
                break

    def on_low_h_thresh_track_bar(self, val):
        """
        Handles the low_h track bar
        :param val: value of low_h
        :return: void
        """
        low_h = val
        low_h = min(self.high_H - 1, low_h)
        cv2.setTrackbarPos(self.low_H_name, self.window_detection_name, low_h)

    def on_high_h_thresh_track_bar(self, val):
        """
        Handles the high_h track bar
        :param val: value of high_h
        :return: void
        """
        high_h = val
        high_h = max(high_h, self.low_H + 1)
        cv2.setTrackbarPos(self.high_H_name, self.window_detection_name, high_h)

    def on_low_s_thresh_track_bar(self, val):
        """
        Handles the low_s track bar
        :param val: value of low_s
        :return: void
        """
        low_s = val
        low_s = min(self.high_S - 1, low_s)
        cv2.setTrackbarPos(self.low_S_name, self.window_detection_name, low_s)

    def on_high_s_thresh_track_bar(self, val):
        """
        Handles the high_s track bar
        :param val: value of high_s
        :return: void
        """
        high_s = val
        high_s = max(high_s, self.low_S + 1)
        cv2.setTrackbarPos(self.high_S_name, self.window_detection_name, high_s)

    def on_low_v_thresh_track_bar(self, val):
        """
        Handles the low_v track bar
        :param val: value of low_v
        :return: void
        """
        low_v = val
        low_v = min(self.high_V - 1, low_v)
        cv2.setTrackbarPos(self.low_V_name, self.window_detection_name, low_v)

    def on_high_v_thresh_track_bar(self, val):
        """
        Handles the high_v track bar
        :param val: value of high_v
        :return: void
        """
        high_v = val
        high_v = max(high_v, self.low_V + 1)
        cv2.setTrackbarPos(self.high_V_name, self.window_detection_name, high_v)


if __name__ == "__main__":
    HSVColorTest()
