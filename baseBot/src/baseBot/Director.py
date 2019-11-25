#!/usr/bin/env python
from data.Task import Task
from data.Robot import Robot
from geometry_msgs.msg import Pose
from baseBot.srv import RequestCleanTask, RequestTask, PassDumpTask, Identify
from baseBot.msg import AvoidAlert, ZoneMSG
from support.Constants import *
import rospy


class Director:
    def __init__(self, robot_manager, cleaning_manager):
        self.robotManager = robot_manager
        self.cleaningManager = cleaning_manager
        self.pub = None
        self.ros_node()
        pass

    # Set up ROS initiators
    def ros_node(self):
        rospy.init_node('base_bot_director', anonymous=True)
        s = rospy.Service('give_zones', RequestTask, self.give_cleaning_task)
        s = rospy.Service('identify_worker', Identify, self.give_ID)
        s = rospy.Service('dump_request', RequestTask, self.handle_dump_request)
        s = rospy.Service('give_avoid_status', RequestTask, self.give_avoid_status)
        print("Director node started")

    # The following methods will create Tasks to send to the small bots that will move them

    # sends an Avoid Task to move back to its directed zone
    def go_back_to_zone(self, workerID):
        task = Task()
        robot = self.robotManager.get_robot(workerID)
        zone = robot.task.zone

        pass

    # sends a Task to robot with given ID to divert off current path in order to avoid another robot
    def avoid_to_direction(self, direction, workerID):
        task = Task()
        pass

    def give_avoid_status(self, robot_request):
        task = Task()
        if self.robotManager.should_robot_avoid(robot_request.workerID):
            if DEBUG:
                print("sending avoid task to robot with ID:")
                print(robot_request.workerID)
            task.make_avoid_task(Pose(), robot_request.workerID)
            return task.to_service_format()
        else:
            if DEBUG:
                print("sending safe task to robot with ID:")
                print(robot_request.workerID)
            task.make_safe_task(robot_request.workerID)
            return task.to_service_format()

    # sends a Task to a robot to not move in order for another robot to avoid it
    # only called when all directions are unsafe to avoid to
    def play_possum(self, workerID):
        task = Task()
        pass

    # identifies new robot and gives them a worker ID
    def give_ID(self, robot_id_request):
        if robot_id_request.ID is -1:
            print("giving new robot id:")
            new_workerID = len(self.robotManager.managedRobots) + 1
            print(new_workerID)
            self.robotManager.add_new_robot(new_workerID)
            return new_workerID
        else:
            return robot_id_request.ID

    def give_cleaning_task(self, robot_request):
        if len(self.cleaningManager.cleaningTasks) > 0:
            print("giving cleaning task")
            task = self.cleaningManager.cleaningTasks.pop()
            task.workerID = robot_request.workerID
            return task.to_service_format()
        else:
            print("no cleaning Task")
            task = Task()
            task.make_safe_task(robot_id)
            return task.to_service_format()

    def handle_dump_request(self, dump_request):
        self.cleaningManager.dumpRequests.append(dump_request)
        if len(self.cleaningManager.dumpRequests) != 1:
            return True  # wait
        else:
            return False  # go ahead
