#!/usr/bin/python
# title           :TaskManager.py
# description     :identifies and executes tasks for smallbot
# author          :Sean Tidd
# date            :2020-02-11
# version         :0.1
# notes           :
# python_version  :3.5
# ==============================================================================
from data.Task import Task
from small_bot.CleanManager import CleanManager
from small_bot.AvoidManager import AvoidManager
from small_bot.DumpManager import DumpManager
import geometry_msgs.msg
import rospy

class TaskManager:

    def __init__(self, smallbot):
        self.cleanManager = CleanManager(smallbot, self)
        self.dumpManager = DumpManager(smallbot, self)
        self.avoidManager = AvoidManager(smallbot, self)
        self.velPub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

    def do_task(self, task):
        """
        ID the Task and execute it based on type
        :param task: The Task to be executed
        :return: Updated Task of where it left off on execution
        """
        if task.type == "clean":
            return self.cleanManager.do_task(task)
        elif task.type == "dump":
            return self.dumpManager.do_task(task)
        elif task.type == "avoid":
            return self.avoidManager.do_task(task)
        else:
            return task


    #def pub_vel(self, vel):
        """
        Publishes the velocity along the /cmd_vel topic
        :param vel: the velocity 
        :return: 
        """
     #   self.velPub.publish(vel)
