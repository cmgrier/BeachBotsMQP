#!/usr/bin/env python3
from edgetpu.detection.engine import DetectionEngine
from pathlib import Path


class ModelScript:
    def __init__(self):
        """
        Initialization
        """

        self.model_path = Path("Google_Model/model.tflite")

        # load the Google Coral object detection model
        print("[INFO] loading Coral model...")
        self.model = DetectionEngine(self.model_path)

    def detect(self, frame, threshold):
        """
        Detection function
        :param frame: the input frame
        :param threshold: the threshold
        :return: results
        """

        return self.model.DetectWithImage(frame, threshold=threshold, keep_aspect_ratio=True, relative_coord=False)

