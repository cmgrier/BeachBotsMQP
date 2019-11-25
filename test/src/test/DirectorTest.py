#!/usr/bin/env python

import rospy
from baseBot.CleaningManager import CleaningManager
from small_bot.SmallBotManager import SmallBotManager
from test.srv import RequestCleanTask, PassAvoidTask, PassDumpTask, Identify
from geometry_msgs.msg import Pose
from data.Task import Task
from data.Zone import Zone


if __name__ == "__main__":
    SB = SmallBotManager()
#Test ID service request
    while(SB.id == -1):
         x = True
    assert SB.id == 1

#Test get clean task service request
    assert len(SB.tasks) is 0
    SB.taskSeeker.request_clean_task()
    while(len(SB.tasks) is 0):
        x = True
    assert len(SB.tasks) is 1

"""
#Test give avoid service request
    assert len(SB.tasks) is 2
    task = SB.tasks.get()
    assert task.isActive == True
    assert task.isComplete == False
    assert task.workerID is 1
    assert task.zone == 120
    assert task.type == "avoid"


#Test send to ID
    print("**************************")
    print(len(SB.tasks))
    SB.TaskSeeker.send_to_id()
    assert len(SB.TaskSeeker.Tasks) is 1
    print(len(SB.TaskSeeker.Tasks))
"""