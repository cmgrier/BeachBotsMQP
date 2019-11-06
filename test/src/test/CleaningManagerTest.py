#!/usr/bin/env python
import math

from baseBot.CleaningManager import CleaningManager
from data.Robot import Robot
from support.Constants import *

# for testing
if __name__ == '__main__':
    robots = [Robot(0), Robot(1), Robot(2)]
    cleaningManager = CleaningManager(robots)
    map = [0] * 10000  # 100 by 100 height map of 0s
    map_width = int(math.sqrt(len(map)))
    cleaningManager.mapManager.mapMaker.map = map
    cleaningManager.mapManager.update_map()
    cleaningManager.create_cleaning_tasks()

    assert len(cleaningManager.cleaningTasks) == map_width/ZONE_WIDTH


    #TODO Chris you need to add this function to the CleaningManager Class
    #cleaningManager.assign_available_robots()

    for i in range(0, len(cleaningManager.cleaningTasks)):
        if i < len(robots) - 1:
            assert cleaningManager.robotManager.managedRobots[i].isBusy is True

    print("all tests passed")

