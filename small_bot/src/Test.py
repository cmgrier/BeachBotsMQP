#!/usr/bin/env python
import rospy
from data.Task import Task


class Test:
    def __init__(self):
        rospy.init_node('test_for_seeker', Anonymous=True)
        s = rospy.Service('give_zones', ,gives_zones_handler)
        self.robotList = []
        self.Tasks = []
        self.latestTask = Task(0)

    def give_zones_handler(self, robo_id):
        self.latestTask = Task(3)
        if robo_id is -1:
            self.Tasks.append("Robot")
            self.latestTask.workerID = len(self.Tasks)
        else:
            self.latestTask.workerID = robo_id
        return self.latestTask


