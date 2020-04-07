#!/usr/bin/env python
from edgetpu.detection.engine import DetectionEngine
from PIL import Image
from pathlib import Path


class ModelScript:
    def __init__(self):
        """
        Initialization
        """

        self.model_path = Path("model.tflite")

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

    def image(self, frame):
        """
        PIL Stuff
        :param frame: frame
        :return: image
        """

        return Image.fromarray(frame)
