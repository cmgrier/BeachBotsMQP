#!/usr/bin/env python
from small_bot.TaskSeeker import TaskSeeker
from test.srv import RequestCleanTask, Identify, PassDumpTask, PassAvoidTask
from data.Task import Task
import rospy
from small_bot.SmallBotManager import SmallBotManager
from geometry_msgs.msg import Pose

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
    if robo_id.ID == -1:
        print("LOOKS LIKE ID WAS ORIGINALLY -1")
        workerID = len(robotList) + 1
        robotList.append("Robot: " + str(workerID))
        return workerID
    else:
        print("ID WAS NOT ORIGINALLY -1")
        return robo_id.ID


def give_zones_handler( robo_id):
        """
        Service for giving cleaning zones to smallvot amd distrivuting IDs to new smallvots
        :param robo_id: ID of the small rovot requesting a clean Task
        :return: fields for a clean Task
        """
        latestTask = Task(3)
        latestTask.workerID = robo_id.workerID
        start = Pose()
        start.position.x = 30
            # ['isActive', 'isComplete', 'workerID', 'type', 'zone', 'startingPoint']
        return [True, False, latestTask.workerID, "clean", 123, start]

def order_avoid( ID, zone):
        """
        Client for sendimg avoid Tasks to two smallbots
        :param ID: ID of robot
        :param zone:
        :return:
        """
        rospy.wait_for_service("robot_avoid_"+str(ID))
        try:
            give_request = rospy.ServiceProxy('robot_avoid_' + str(ID), PassAvoidTask)
            start = Pose()
            start.position.y = 30
            # ['isActive', 'isComplete', 'workerID', 'type', 'zone', 'startingPoint']
            return [True, False, ID, "avoid", zone, start]

        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)


def order_dump(ID, zone):
        """
        Client for sendimg avoid Tasks to two smallbots
        :param ID: ID of robot
        :param zone:
        :return:
        """
        rospy.wait_for_service("robot_dump_"+str(ID))
        try:
            give_request = rospy.ServiceProxy('robot_dump_' + str(ID), PassDumpTask)
            start = Pose()
            start.position.z = 30
            # ['isActive', 'isComplete', 'workerID', 'type', 'zone', 'startingPoint']
            return [True, False, ID, "dump", zone, start]
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)



if __name__ == "__main__":
    ros_node()
    sbm = SmallBotManager()
    sbm.TaskSeeker.request_ID()
#Test ID service request
    while(sbm.id == -1):
         x = True
    assert sbm.id == 1

#Test get clean task service request
    assert len(sbm.tasks) is 0
    sbm.TaskSeeker.request_clean_task()
    while(len(sbm.tasks) is 0):
        x = True
    assert len(sbm.tasks) is 1


#Test add avoid service request

    order_avoid(1, 123)
    sbm.TaskSeeker.wait_for_service("robot_avoid_1")
    order_avoid(1, 123)
    print (sbm.tasks)
    assert len(sbm.tasks) is 2