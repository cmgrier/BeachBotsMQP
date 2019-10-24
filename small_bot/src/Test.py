#!/usr/bin/env python
import rospy
from data.Task import Task
from data.srv import *

class Test:
    def __init__(self):
        self.robotList = []
        self.Tasks = []
        self.latestTask = Task(0)
        self.ros_node()



    def give_zones_handler(self, robo_id):
        """
        Service for giving cleaning zones to smallvot amd distrivuting IDs to new smallvots
        :param robo_id: ID of the small rovot requesting a clean Task
        :return: fields for a clean Task
        """
        rospy.loginfo("rovo_ID Param")
        rospy.loginfo(robo_id)

        self.latestTask = Task(3)

        if robo_id.workerID is -1:
            self.Tasks.append("Robot")
            self.latestTask.workerID = len(self.Tasks)
            rospy.loginfo(self.latestTask.workerID)
            rospy.loginfo("Part 1")

        else:
            self.latestTask.workerID = robo_id.workerID

            rospy.loginfo("Part 2")
        rospy.loginfo("rovot list:")
        rospy.loginfo(len(self.Tasks))
        return [True, False, self.latestTask.workerID, 123, 3]


    # TODO after testing make sure this operates with 2 IDs and 2 Zones
    def order_avoid(self, ID, zone):
        """
        Client for semdimg avoid Tasks to two smallvots
        :param ID: ID of rovot
        :param zone:
        :return:
        """
        rospy.wait_for_service('robo_avoid_' + ID)
        print("tryimg to request")
        try:
            give_request = rospy.ServiceProxy('robo_avoid_' + ID, GiveAvoidTask)
            send_avoid = give_request([True, False, ID, zone, 1])

        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)
    

    def ros_node(self):
        """
        Sets up ROS initiators
        :return:
        """
        rospy.init_node('test_for_seeker', anonymous=True)
        s = rospy.Service('give_zones', GetCleanTask, self.give_zones_handler)
        print("ros node started")
        rospy.spin()

if __name__=="__main__":
    test = Test()
    test.order_avoid(2,140)