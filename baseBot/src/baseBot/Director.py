#!/usr/bin/env python
import math

from data.Task import Task
from data.Robot import Robot
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from baseBot.srv import RequestCleanTask, RequestTask, PassDumpTask, Identify
from baseBot.msg import AvoidAlert, ZoneMSG
from support.Constants import *
import rospy
import numpy as np


class Director:
    def __init__(self, robot_manager, cleaning_manager):
        self.robotManager = robot_manager
        self.cleaningManager = cleaning_manager
        self.OGpub = None
        self.ros_node()
        pass

    # Set up ROS initiators
    def ros_node(self):
        rospy.init_node('base_bot_director', anonymous=True)
        s = rospy.Service('give_zones', RequestTask, self.give_cleaning_task)
        s = rospy.Service('identify_worker', Identify, self.give_ID)
        s = rospy.Service('dump_request', RequestTask, self.handle_dump_request)
        s = rospy.Service('give_avoid_status', RequestTask, self.give_avoid_status)
        self.OGpub = rospy.Publisher('occupancy_grid', OccupancyGrid, queue_size=10)

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
            task.make_safe_task(robot_request.workerID)
            return task.to_service_format()

    def handle_dump_request(self, dump_request):
        self.cleaningManager.dumpRequests.append(dump_request)
        if len(self.cleaningManager.dumpRequests) != 1:
            return True  # wait
        else:
            return False  # go ahead

    def publish_og(self):
        og = self.cleaningManager.mapManager.occupancy_grid
        OG = OccupancyGrid()
        altered_og = []
        for val in og:
            if val == -1:
                altered_og.append(val)
            else:
                altered_og.append(int(val * 100 - 70))
        OG.data = altered_og
        for i in range(0, 3):
            OG.data[i] = 99
        OG.info.width = int(math.sqrt(len(og)))
        OG.info.height = int(math.sqrt(len(og)))
        OG.info.resolution = (2 * X_MAX) / OG_WIDTH
        OG.info.origin = self.__get_og_origin()
        self.OGpub.publish(OG)

    def handle_og_request(self, og_request):
        og = self.cleaningManager.mapManager.occupancy_grid
        OG = OccupancyGrid()
        altered_og = []
        for val in og:
            if val == -1:
                altered_og.append(val)
            else:
                altered_og.append(int(val * 100 - 70))
        OG.data = altered_og
        OG.info.width = int(math.sqrt(len(og)))
        OG.info.height = int(math.sqrt(len(og)))
        OG.info.resolution = OG_WIDTH / (2 * X_MAX)
        origin = Pose()
        o_quat = self.cleaningManager.mapManager.mapMaker.orientation
        o_euler = euler_from_quaternion(o_quat)
        z_angle = o_euler[2]

        origin.position.x = self.cleaningManager.mapManager.mapMaker.translation[0]
        origin.position.y = self.cleaningManager.mapManager.mapMaker.translation[1]
        origin.position.z = self.cleaningManager.mapManager.mapMaker.translation[2]
        OG.info.origin = origin

    def __get_og_origin(self):
        origin = Pose()
        zed_quat = self.cleaningManager.mapManager.mapMaker.orientation
        zed_euler = self.quaternion_to_euler(zed_quat[0], zed_quat[1], zed_quat[2], zed_quat[3])

        angle_r = math.atan2(X_MAX, Y_MAX)

        heading = zed_euler[2] + angle_r

        hypotenuse = math.sqrt(math.pow(X_MAX, 2) + math.pow(Y_MAX, 2))

        origin.position.x = self.cleaningManager.mapManager.mapMaker.translation[0] - hypotenuse * math.sin(heading)
        origin.position.y = self.cleaningManager.mapManager.mapMaker.translation[1] + hypotenuse * math.cos(heading)
        origin.position.z = self.cleaningManager.mapManager.mapMaker.translation[2]

        o_quat = self.euler_to_quaternion(zed_euler[2], zed_euler[1], zed_euler[0] + math.radians(180))

        origin.orientation.x = o_quat[0]
        origin.orientation.y = o_quat[1]
        origin.orientation.z = o_quat[2]
        origin.orientation.w = o_quat[3]
        return origin

    def quaternion_to_euler(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)

        return [qx, qy, qz, qw]