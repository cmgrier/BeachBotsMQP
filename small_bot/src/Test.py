#!/usr/bin/env python
import sys
import rospy
from data.Task import Task
from small_bot.srv import RequestCleanTask, Identify, PassAvoidTask

class Test:
    def __init__(self):
        self.robotList = []
        self.Tasks = []
        self.latestTask = Task(0)
        self.ros_node()
        rospy.sleep(8)
        self.order_avoid(2, 360)



    def give_ID_handler(self, robo_id):
        """
        Service for giving a small_vot an ID and adding it to the workforce list
        :param robo_id: current ID of the small_vot
        :return: a next avaivle rovot ID numver
        """
        if robo_id.ID is -1:
            workerID = len(self.robotList) + 1
            self.robotList.append("Robot: " + str(workerID))
            return workerID
        else:
            return robo_id.ID


    def give_zones_handler(self, robo_id):
        """
        Service for giving cleaning zones to smallvot amd distrivuting IDs to new smallvots
        :param robo_id: ID of the small rovot requesting a clean Task
        :return: fields for a clean Task
        """
        self.latestTask = Task(3)

        if robo_id.workerID is -1:
            self.Tasks.append("Robot")
            self.latestTask.workerID = len(self.Tasks)
        else:
            self.latestTask.workerID = robo_id.workerID
            # [isActive, isComplete, WorkerID, Zone, Task type]
        return [True, False, self.latestTask.workerID, 123, "clean"]


    # TODO after testing make sure this operates with 2 IDs and 2 Zones
    def order_avoid(self, ID, zone):
        """
        Client for semdimg avoid Tasks to two smallvots
        :param ID: ID of rovot
        :param zone:
        :return:
        """
        rospy.wait_for_service("robot_avoid_"+str(ID))
        print("tryimg to request AVOID")
        try:
            give_request = rospy.ServiceProxy('robot_avoid_' + str(ID), PassAvoidTask)
            send_avoid = give_request(True, False, ID, zone, "avoid")

        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)
    

    def ros_node(self):
        """
        Sets up ROS initiators
        :return:
        """
        rospy.init_node('test_for_seeker', anonymous=True)
        s = rospy.Service('give_zones', RequestCleanTask, self.give_zones_handler)
        s = rospy.Service('identify_worker', Identify, self.give_ID_handler)
        print("ros node started")
        #rospy.spin()

if __name__=="__main__":
    test = Test()