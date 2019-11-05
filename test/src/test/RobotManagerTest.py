# for testing
from baseBot.MapManager import MapManager
from data.Robot import Robot

from baseBot.CleaningManager import CleaningManager

if __name__ == '__main__':
    rbt1 = Robot(0)
    robots = [Robot(0), Robot(1), Robot(2)]
    mapManager = MapManager()
    cleaningManager = CleaningManager(robots)
    cleaningManager.robotManager.assign_robot(0)
    assert cleaningManager.robotManager.get_robot(0).isBusy is True
