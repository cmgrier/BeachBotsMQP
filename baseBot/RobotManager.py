import itertools
import math

from baseBot.Director import Director
from baseBot.RobotFinder import RobotFinder
from support import Constants


# manages the robots connected to the base bot
class RobotManager:
    def __init__(self, robots):
        self.managedRobots = robots
        self.robotFinder = RobotFinder()
        self.director = Director()

    # for each robot this checks if they are too close to one another.
    def check_robot_safety(self):
        for robot in self.managedRobots():
            if self.is_out_of_zone(robot.workerID):
                self.director.go_back_to_zone(robot.workerID)
        for robot1, robot2 in itertools.combinations(self.managedRobots, 2):
            if self.__are_robots_too_close(robot1.position, robot2.position):
                if self.left_turn_available(robot1.workerID):
                    self.director.avoid_to_left(robot1.workerID)
                elif self.right_turn_available(robot1.workerID):
                    self.director.avoid_to_right(robot1.workerID)
                elif self.left_turn_available(robot2.workerID):
                    self.director.avoid_to_left(robot2.workerID)
                elif self.right_turn_available(robot2.workerID):
                    self.director.avoid_to_right(robot2.workerID)

    # checks if the given robot positions are too close
    def __are_robots_too_close(self, robot_position1, robot_position2):
        distance = math.hypot(robot_position1.position[0] - robot_position2.position[0],
                              robot_position1.position[1] - robot_position2.position[1])
        return distance < Constants.safe_distance_between_bots

    # updates the positions of the robots
    def update_robot_positions(self):
        new_robot_positions = self.robotFinder.find_robots()
        for new_robot_position in new_robot_positions:
            robot = self.__get_robot(new_robot_position[0])
            robot.position = new_robot_position[1]

    # marks the given robot as busy
    def assign_robot(self, workerID):
        self.__get_robot(workerID).isBusy = True

    def is_out_of_zone(self, workerID):
        robot = self.__get_robot(workerID)
        return robot.task.zone.is_out_of_zone(robot.position)

    def __get_robot(self, workerID):
        for robot in self.managedRobots:
            if robot.workerID == workerID:
                return robot

    def left_turn_available(self, workerID):
        pass
        #todo

    def right_turn_available(self, workerID):
        pass
        #todo
