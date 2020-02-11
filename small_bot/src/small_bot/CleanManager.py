#!/usr/bin/python
# title           :CleanManager.py
# description     :executes clean tasks for smallbot
# author          :Sean Tidd
# date            :2020-02-11
# version         :0.1
# notes           :
# python_version  :3.5
# ==============================================================================
from data.Task import Task
from support.Constants import *
import geometry_msgs.msg

class CleanManager:

    def __init__(self, smallbot, taskManager):
        self.smallbot = smallbot
        self.taskManager = taskManager
        self.counter = 0    # here for testing
        pass

    def do_task(self, task):
        """
        Will attempt to complete the given clean task
        :param task: the clean task to be executed
        :return: the updated task
        """
        self.counter = self.counter + 1
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 60
        self.taskManager.pub_vel(twist)

        if DEBUG:
            print("working on cleaning task")
            print(self.counter)
        if self.counter > 100:
            print("CLEANING TASK COMPLETE")
            self.counter = 0
            task.isComplete = True
        return task
