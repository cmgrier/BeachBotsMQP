#!/usr/bin/env python
import itertools
import math

from baseBot.Director import Director
from baseBot.RobotFinder import RobotFinder
from data.Robot import Robot
from support.Constants import *


# manages the robots connected to the base bot
class RobotManager:
    def __init__(self, robots, map_manager, cleaning_manager):
        self.managedRobots = robots
        self.robotFinder = RobotFinder()
        self.cleaning_manager = cleaning_manager
        self.director = Director(self, cleaning_manager)
        self.map_manager = map_manager

    # for each robot this checks if they are too close to one another.
    def check_robot_safety(self):
        for robot in self.managedRobots():
            if self.__is_out_of_zone(robot.workerID):
                self.director.go_back_to_zone(robot.workerID)
        for robot1, robot2 in itertools.combinations(self.managedRobots, 2):
            if self.__are_robots_too_close(robot1.position, robot2.position):
                self.__divert_robots(robot1, robot2)

    # gives one of the given robots an avoid Task based on which robot a higher priority current task
    def __divert_robots(self, robot1, robot2):
        safe_directions_robot1 = self.__get_safe_escape_directions(robot1.workerID)
        safe_directions_robot2 = self.__get_safe_escape_directions(robot2.workerID)

        if robot2.task.robot_to_avoid is None and robot1.task.robot_to_avoid is None:
            if robot1.task.priority > 2 and safe_directions_robot1:
                self.director.avoid_to_direction(safe_directions_robot1[0], robot1.workerID)
            elif safe_directions_robot2:
                self.director.avoid_to_direction(safe_directions_robot2[0], robot2.workerID)
            else:
                self.director.play_possum(robot1.workerID)
        elif robot1.task.robot_to_avoid != robot2.workerID and robot2.task.robot_to_avoid != robot1.workerID:
            if robot1.task.robot_to_avoid is None and safe_directions_robot1:
                self.director.avoid_to_direction(safe_directions_robot1[0], robot1.workerID)
            elif robot2.task.robot_to_avoid is None and safe_directions_robot2:
                self.director.avoid_to_direction(safe_directions_robot2[0], robot2.workerID)
            elif safe_directions_robot1:
                self.director.avoid_to_direction(safe_directions_robot1[0], robot1.workerID)
            elif safe_directions_robot2:
                self.director.avoid_to_direction(safe_directions_robot2[0], robot2.workerID)
            else:
                self.director.play_possum(robot1.workerID)

    # checks if the given robot positions are too close
    def __are_robots_too_close(self, robot_position1, robot_position2):
        distance = math.hypot(robot_position1.position[0] - robot_position2.position[0],
                              robot_position1.position[1] - robot_position2.position[1])
        return distance < Constants.safe_distance_between_bots

    # updates the positions of the robots
    def update_robot_positions(self):
        new_robot_positions = self.robotFinder.find_robots()
        for new_robot_position in new_robot_positions:
            robot = self.get_robot(new_robot_position[0])
            robot.position = new_robot_position[1]

    # add new robot
    def add_new_robot(self, workerID):
        new_robot = Robot(workerID)
        self.managedRobots.append(new_robot)

    # marks the given robot as busy
    def assign_robot(self, workerID):
        self.get_robot(workerID).isBusy = True

    # checks if robot with given worker ID is outside of its zone
    def __is_out_of_zone(self, workerID):
        robot = self.get_robot(workerID)
        return robot.task.zone.__is_out_of_zone(robot.position)

    # returns the robot that has the given worker ID
    def get_robot(self, workerID):
        for robot in self.managedRobots:
            if robot.workerID == workerID:
                return robot

    # finds which directions are safe to avoid to
    def __get_safe_escape_directions(self, workerID):
        given_bot = self.get_robot(workerID)
        safe_directions = Constants.avoid_direction_priority_list
        for robot in self.managedRobots:
            if robot.workerID != workerID:
                if self.__are_robots_too_close(given_bot.pose, robot.pose):
                    direction = self.__get_other_robot_direction(workerID, robot.workerID)
                    safe_directions.remove(direction)
        safe_directions = self.__check_direction_for_difficult_terrain(given_bot, safe_directions)
        return safe_directions

    # todo will be filled out in the future. Not filled out now to save my sanity
    def __check_direction_for_difficult_terrain(self, robot, directions):
        local_indices = self.__get_local_indices(robot.pose[0])
        return directions

    # built for future use
    def __get_local_indices(self, position):
        local_indices = []
        origin_index = self.__point_to_index(position)
        top_left_index = origin_index - Constants.avoid_distance * (int(math.sqrt(len(self.map_manager.map))) + 1)
        for i in range(0, Constants.avoid_distance * 2):
            for j in range(0, Constants.avoid_distance * 2):
                local_indices.append(int(top_left_index + i * Constants.avoid_distance + j))
        return local_indices

    # converts given point to nearest index
    def __point_to_index(self, point):
        width = int(math.sqrt(len(self.map_manager.map)))
        index = point[0] + point[1] * width
        return index

    # todo should be updated when ROS data types are added
    # returns the angle between two robots based on the firsts heading
    def __get_angle_between_bots(self, start_bot_ID, target_bot_ID):
        start_bot = self.get_robot(start_bot_ID)
        target_bot = self.get_robot(target_bot_ID)
        robot_heading = start_bot.pose[1][2]
        x = target_bot.pose[0][0] - start_bot.pose[0][0]
        y = target_bot.pose[0][0] - start_bot.pose[0][0]
        position_angle = math.atan2(y, x)
        return robot_heading - position_angle

    # straight ahead is 0, right is positive, left is negative, pi = -pi
    # directions are in 8ths around the circle starting with 0 front right to the right
    def __get_other_robot_direction(self, start_bot_ID, target_bot_ID):
        angle = self.__get_angle_between_bots(start_bot_ID, target_bot_ID)
        if angle > 0:
            # right
            if angle < .7854:
                # top
                return 0
            elif angle < 1.5708:
                # mid top
                return 1
            elif angle < 2.3562:
                # mid bottom
                return 2
            else:
                # bottom
                return 3
        # left
        elif angle > -.7854:
            # top
            return 7
        elif angle > -1.5708:
            # mid top
            return 6
        elif angle > -2.3562:
            # mid bottom
            return 5
        else:
            # bottom
            return 4
