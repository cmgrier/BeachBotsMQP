#!/usr/bin/env python
import rospy
from data.Task import Task
from data.srv import GetCleanTask

class Test:
    def __init__(self):
        self.robotList = []
        self.Tasks = []
        self.latestTask = Task(0)
        self.ros_node()



    def give_zones_handler(self, robo_id):
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

    def ros_node(self):
        rospy.init_node('test_for_seeker', anonymous=True)
        s = rospy.Service('give_zones', GetCleanTask, self.give_zones_handler)
        print("ros node started")
        rospy.spin()

if __name__=="__main__":
    test = Test()
