#!/usr/bin/env python
from small_bot.TaskSeeker import TaskSeeker
from test.srv import RequestCleanTask, Identify, PassDumpTask, PassAvoidTask
from data.Task import Task
import rospy



#Test variables
Tasks = []
robotList = []

def ros_node():
    """
    Sets up ROS initiators
    :return:
    """
    rospy.init_node('task_seeker', anonymous=True)
    s = rospy.Service('give_zones', RequestCleanTask, give_zones_handler)
    s = rospy.Service('identify_worker', Identify, give_ID_handler)


def give_ID_handler( robo_id):
    """
    Service for giving a small_vot an ID and adding it to the workforce list
    :param robo_id: current ID of the small_vot
    :return: a next avaivle rovot ID numver
    """
    if robo_id.ID is -1:
        workerID = len(robotList) + 1
        robotList.append("Robot: " + str(workerID))
        return workerID
    else:
        return robo_id.ID


def give_zones_handler( robo_id):
        """
        Service for giving cleaning zones to smallvot amd distrivuting IDs to new smallvots
        :param robo_id: ID of the small rovot requesting a clean Task
        :return: fields for a clean Task
        """
        latestTask = Task(3)
        if robo_id.workerID is -1:
            Tasks.append("Robot")
            latestTask.workerID = len(Tasks)
        else:
            latestTask.workerID = robo_id.workerID
            # [isActive, isComplete, WorkerID, Zone, Task type]
        return [True, False, latestTask.workerID, 123, "clean"]

def order_avoid( ID, zone):
        """
        Client for semdimg avoid Tasks to two smallvots
        :param ID: ID of rovot
        :param zone:
        :return:
        """
        rospy.wait_for_service("robot_avoid_"+str(ID))
        try:
            give_request = rospy.ServiceProxy('robot_avoid_' + str(ID), PassAvoidTask)
            send_avoid = give_request(True, False, ID, zone, "avoid")

        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)


def order_dump( ID, zone):
        """
        Client for semdimg avoid Tasks to two smallvots
        :param ID: ID of rovot
        :param zone:
        :return:
        """
        rospy.wait_for_service("robot_dump_"+str(ID))
        try:
            give_request = rospy.ServiceProxy('robot_dump_' + str(ID), PassDumpTask)
            send_dump = give_request(True, False, ID, zone, "dump")

        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)



if __name__ == "__main__":
    ros_node()
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
