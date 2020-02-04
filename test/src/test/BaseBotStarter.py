#!/usr/bin/env python3

import rospy
from baseBot.CleaningManager import CleaningManager
from geometry_msgs.msg import Pose
from data.Task import Task
from data.Zone import Zone
import time
import pyzed.sl as sl

def zed_test():
    emptyRobotList = []
    taskID = 0
    CM = CleaningManager(emptyRobotList)
    print("Getting Frames...")
    timer = 0
    # Grab 500 frames and stop
    while timer < 500:
        CM.mapManager.mapMaker.update_map_async()
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1

    CM.mapManager.update_zones()
    CM.create_cleaning_tasks()
    print(len(CM.cleaningTasks))
    print(CM.cleaningTasks[0])
    print(CM.cleaningTasks[0].zone.corners)

def basic_test():
    emptyRobotList = []
    taskID = 0
    CM = CleaningManager(emptyRobotList)
    cleaningTask1 = Task(Zone([Pose(), Pose(), Pose(), Pose()], taskID))
    CM.cleaningTasks.append(cleaningTask1)
    while 1:
        if len(CM.cleaningTasks) < 1:
            taskID = taskID + 1
            cleaningTask = Task(Zone([Pose(), Pose(), Pose(), Pose()], taskID))
            CM.cleaningTasks.append(cleaningTask)
            print("added new cleaning task with ID:")
            print(taskID)
        time.sleep(.5)
        # rospy.sleep(.01)

def og_test():
    emptyRobotList = []
    CM = CleaningManager(emptyRobotList)
    timer = 0
    print("Getting Frames...")
    while timer < 500:
        CM.mapManager.mapMaker.update_map_async()
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1
    CM.mapManager.update_OG()
    while 1:
        CM.robotManager.director.publish_og()
        rospy.sleep(.01)


if __name__ == "__main__":
    og_test()