#!/usr/bin/env python


# this class will hold all of the other managers and relay information between them.
# It will also hold on to the queued cleaning tasks
from baseBot.MapManager import MapManager


class CleaningManager:
    def __init__(self, robots):
        self.mapManager = MapManager()
        self.cleaningTasks = []
        self.robotManager = RobotManager(robots, self.mapManager, self)

        self.create_cleaning_tasks()

    # run on startup
    def create_cleaning_tasks(self):
        for zone in self.mapManager.zones:
            self.cleaningTasks.append(Task(zone=zone))
