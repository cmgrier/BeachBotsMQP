# for testing
from baseBot.MapManager import MapManager
from baseBot.RobotManager import RobotManager
from data.Robot import Robot

if __name__ == '__main__':
    robots = [Robot(0), Robot(1), Robot(2)]
    mapManager = MapManager()
    robotManager = RobotManager(robots, mapManager)
