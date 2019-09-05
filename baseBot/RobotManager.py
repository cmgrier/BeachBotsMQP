import itertools
import math

from baseBot.RobotFinder import RobotFinder
from support import Constants


# manages the robots connected to the base bot
class RobotManager:
    def __init__(self, robots):
        self.managedRobots = robots
        self.robotPositions = list()
        self.robotFinder = RobotFinder()

    # for each robot this checks if they are too close to one another.
    def check_robot_safety(self):
        for robotPosition1, robotPosition2 in itertools.combinations(self.robotPositions, 2):
            if self.are_robots_too_close(robotPosition1, robotPosition2):
                print ("fix trajectory of " + robotPosition1.robot + " and " + robotPosition2.robot)
                #fix trajectory todo

    # checks if the given robot positions are too close
    def are_robots_too_close(self, robotPosition1, robotPosition2):
        distance = math.hypot(robotPosition1.position[0] - robotPosition2.position[0], robotPosition1.position[1] - robotPosition2.position[1])
        return distance < Constants.safe_distance_between_bots

    # updates the positions of the robots
    def update_robot_positions(self):
        newRobotPositions = self.robotFinder.find_robots()
        self.robotPositions = newRobotPositions

    # marks the given robot as busy
    def assign_robot(self, robot):
        for bot in self.managedRobots:
            if bot.workerID == robot.workerID:
                bot.isBusy = True