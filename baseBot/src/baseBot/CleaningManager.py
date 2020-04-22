#!/usr/bin/env python


# this class will hold all of the other managers and relay information between them.
# It will also hold on to the queued cleaning tasks
from baseBot.MapManager import MapManager
from baseBot.RobotManager import RobotManager
from data.Task import Task


class CleaningManager:
    def __init__(self, robots):
        self.mapManager = MapManager()
        self.cleaningTasks = []
        self.robotManager = RobotManager(robots, self.mapManager, self)
        self.dumpRequests = []
        self.completed_tasks = []

        self.create_cleaning_tasks()

    # run on startup and repeatedly to continuously create cleaning tasks for new zones
    # creates
    def create_cleaning_tasks(self):
        for zone in self.mapManager.zones:
            exists = False
            for task in self.cleaningTasks:
                if task.zone.id == zone.id:
                    exists = True
                    break
            for task in self.completed_tasks:
                if task.zone.id == zone.id:
                    exists = True
                    break
            if not exists:
                self.cleaningTasks.append(Task(zone=zone))

