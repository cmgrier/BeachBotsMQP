#!/usr/bin/env python

from baseBot.CleaningManager import CleaningManager
from small_bot.TaskSeeker import TaskSeeker
from test.srv import RequestCleanTask, PassAvoidTask, PassDumpTask, Identify
from data.Task import Task
from data.Zone import Zone
import rospy

if __name__ == "__main__":
    emptyRobotList = []
    CM = CleaningManager(emptyRobotList)
    cleaningTask1 = Task(Zone([1,1,1,1], 0))
    CM.cleaningTasks.append(cleaningTask1)
    cleaningTask2 = Task(Zone([1, 1, 1, 1], 1))
    CM.cleaningTasks.append(cleaningTask2)
    TS = TaskSeeker()
#Test ID service request
    while(TS.ID == -1):
         x = True
    assert TS.ID == 1

#Test get clean task service request
    assert len(TS.Tasks) is 0
    TS.request_clean_task()
    while(len(TS.Tasks) is 0):
        x = True
    assert len(TS.Tasks) is 1

#Test give avoid service request
    order_avoid(1, 120)
    assert len(TS.Tasks) is 2
    task =  TS.Tasks.get()
    assert task.isActive == True
    assert task.isComplete == False
    assert task.workerID is 1
    assert task.zone == 120
    assert task.type == "avoid"

#Test send to ID
    print("**************************")
    print(len(TS.Tasks))
    TS.send_to_id()
    assert len(TS.Tasks) is 1
    print(len(TS.Tasks))