#!/usr/bin/env python

import rospy
from baseBot.CleaningManager import CleaningManager
from small_bot.TaskSeeker import TaskSeeker
from test.srv import RequestCleanTask, PassAvoidTask, PassDumpTask, Identify
from geometry_msgs.msg import Pose
from data.Task import Task
from data.Zone import Zone

if __name__ == "__main__":
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
        rospy.sleep(.5)