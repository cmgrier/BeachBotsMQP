#!/usr/bin/python
# title           :Task.py
# description     :main node for smallbot
# author          :Sean Tidd
# date            :2020-02-11
# version         :0.1
# notes           :
# python_version  :3.5
# ==============================================================================
from geometry_msgs.msg import Pose
from data.Zone import Zone

class Task:
    def __init__(self, zone=None, type=None):
        self.isActive = False
        self.isComplete = False
        self.workerID = -1
        self.type = type or "clean"
        self.zone = zone or Zone([], -1)
        self.priority = 3
        self.start_point = Pose()
        self.set_priority()

    # set the priority level of the task based on the type
    def set_priority(self):
        """
        Automatically set the priority of the task by type
        :return:
        """
        if self.type == "clean":
            self.priority = 3
        elif self.type == "avoid":
            self.priority = 1
        elif self.type == "dump":
            self.priority = 2
        else:
            self.priority = 4

    def __repr__(self):
        return str((self.type, self.workerID, self.zone, self.isActive, self.isComplete))

    def __str__(self):
        return str((self.type, self.workerID, self.zone, self.isActive, self.isComplete))

    def to_service_format(self):
        return [self.isActive, self.isComplete, self.workerID, self.type, self.zone.corners, self.zone.id, self.start_point]

    def make_safe_task(self, worker_id):
        self.isActive = False
        self.isComplete = False
        self.type = "safe"
        self.zone = Zone([], -1)
        self.priority = 4
        self.start_point = Pose()
        self.workerID = worker_id

    def make_avoid_task(self, goal, worker_id):
        self.isComplete = False
        self.isActive = False
        self.type = "avoid"
        self.set_priority()
        self.zone = Zone([], -1)
        self.workerID = worker_id
        self.start_point = goal
