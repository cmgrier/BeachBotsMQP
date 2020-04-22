#!/usr/bin/env python3
from ModelScript import ModelScript
from CoralMain import CoralMain


class CV_Connector:
    def __init__(self):

        self.model_script = ModelScript()
        self.coral_main = CoralMain(self.model_script)


if __name__ == "__main__":
    cv_con = CV_Connector()
