#!/usr/bin/python
# title           :AvoidManager.py
# description     :executes avoid tasks for smallbot
# author          :Sean Tidd
# date            :2020-02-11
# version         :0.1
# notes           :
# python_version  :3.5
# ==============================================================================
from data.Task import Task
from support.Constants import *
import geometry_msgs.msg


class AvoidManager:

    def __init__(self, smallbot, taskManager):
        self.smallbot = smallbot
        self.taskManager = taskManager
        self.counter = 0
        self.avoid_test = False
        pass

    def do_task(self, task):
        """
        Will attempt to complete the given avoid task
        :param task: the avoid task to be executed
        :return: the updated task
        """
        self.counter = self.counter + 1
        twist = geometry_msgs.msg.Twist()
        twist.angular.z = 40
        self.taskManager.pub_vel(twist)
        if DEBUG:
            print("working on avoid task")
            print(self.counter)
        if self.avoid_test and self.counter > 100:
            print("AVOID TASK COMPLETE")
            self.counter = 0
            task.isComplete = True
        else:
            #TODO: IMPLEMENT THE CODE TO MAKE THE ROBOT NAVIGATE TO THE COORD VIA NAVIGATE::nav_to_coord()
            #TODO: IF THE SMALLBOT REACHES THE COORD, SET THE TASK ISCOMPLETE TO TRUE
            print()
        return task

