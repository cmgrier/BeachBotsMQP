#!/usr/bin/env python
from data.Task import Task
from data.Robot import Robot
from baseBot.srv import RequestCleanTask, PassAvoidTask, PassDumpTask, Identify
from baseBot.msg import AvoidAlert
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
        # s = rospy.Service('give_zones', RequestCleanTask, self.give_cleaning_task)
        # s = rospy.Service('identify_worker', Identify, self.give_ID)
        # s = rospy.Service('dump_request', PassDumpTask, self.handle_dump_request)
        self.pub = rospy.Publisher('avoid_alert_1', AvoidAlert, queue_size=10)
        print("ros node started")

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

    def send_avoid(self, goal, workerID):
        task = Task()
        task.make_avoid_task(goal, workerID)
        topic_name = 'avoid_alert_' + str(workerID)
        msg = AvoidAlert()
        msg.isActive = task.isActive
        msg.isComplete = task.isComplete
        msg.workerID = task.workerID
        msg.type = task.type
        msg.zone = task.zone
        msg.startingPoint = task.start_point
        self.pub.publish(msg)
        # print(topic_name)
        # print("end of send_avoid method")
        pass

    def send_safe(self, workerID):
        task = Task()
        task.make_safe_task(workerID)
        topic_name = 'avoid_alert_' + str(workerID)
        msg = AvoidAlert()
        msg.isActive = task.isActive
        msg.isComplete = task.isComplete
        msg.workerID = task.workerID
        msg.type = task.type
        msg.zone = task.zone
        msg.startingPoint = task.start_point
        self.pub.publish(msg)
        # print(topic_name)
        # print("end of send_avoid method")
        pass

    # sends a Task to a robot to not move in order for another robot to avoid it
    # only called when all directions are unsafe to avoid to
    def play_possum(self, workerID):
        task = Task()
        pass

    # identifies new robot and gives them a worker ID
    def give_ID(self, robot_id_request):
        if robot_id_request.ID is -1:
            new_workerID = len(self.robotManager.managedRobots) + 1
            self.robotManager.add_new_robot(new_workerID)
            return new_workerID
        else:
            return robot_id_request.ID

    def give_cleaning_task(self, robot_id):
        task = self.cleaningManager.cleaningTasks.pop()
        if task:
            task.workerID = robot_id
            return task.to_service_format()
        else:
            return Task(None).to_service_format()

    def handle_dump_request(self, dump_request):
        self.cleaningManager.dumpRequests.append(dump_request)
        if len(self.cleaningManager.dumpRequests) != 1:
            return True  # wait
        else:
            return False  # go ahead
