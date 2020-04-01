#!/usr/bin/python
# title           :DumpManager.py
# description     :executes dump tasks for smallbot
# author          :Sean Tidd
# date            :2020-02-11
# version         :0.1
# notes           :
# python_version  :3.5
# ==============================================================================
from data.Task import Task


class DumpManager:

    def __init__(self, smallbot, taskManager):
        self.smallbot = smallbot
        self.taskManager = taskManager
        pass

    def do_task(self, task):
        """
        Will attempt to complete the given dump task
        :param task: the dump task to execute
        :return: the updated task
        """
        return task
