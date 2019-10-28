#!/usr/bin/env python
import rospy
from std_msgs.msg import String
#from smallBot.TaskIdentifier import TaskIdentifier TODO optimize of ROS
from data.Task import Task
from small_bot.srv import RequestCleanTask, PassAvoidTask, Identify
from support.EqualPriorityQueue import EqualPriorityQueue


"""
This class resquest a new task and sends that task off to be identified.
This class also updates the status for the avoid and return tasks
"""
class TaskSeeker:

    def __init__(self):
        rospy.init_node('task_seeker', anonymous=True)
        #self.TaskIdentifier = TaskIdentifier() TODO REMOVE QOUTES
        self.currentTask = None
        self.Tasks = EqualPriorityQueue()
        self.ID = -1
        self.request_ID()
        #self.request_clean_task()
        rospy.spin()
       # self.send_to_id() TODO REMOVE after TaskID Update


    def parse_task(self, taskResponse):
        """
        Takes a service response amd comvines all the fields into a Task
        :param taskResponse: response received from a service
        :return: a Task
        """
        task = Task(taskResponse.zone, taskResponse.type)
        task.isActive = taskResponse.isActive
        task.isComplete = taskResponse.isComplete
        task.workerID = taskResponse.workerID
        return task


    def request_ID(self):
        """
        Client that asks for a robot ID from basebot
        :return:
        """
        rospy.wait_for_service('identify_worker')
        print("trying to request")
        try:
            rospy.loginfo("Current ID: " + str(self.ID))
            request = rospy.ServiceProxy('identify_worker', Identify)
            service_result = request(self.ID)
            self.ID = service_result.newID
            rospy.loginfo("new ID: " + str(self.ID))
            topic_name = "robot_avoid_"+str(self.ID)
            s = rospy.Service(topic_name, PassAvoidTask, self.update_avoid_status_handler)

        except rospy.ServiceException, e:
            rospy.loginfo("Service call (request_ID) failed: %s" % e)


    def request_clean_task(self):
        """
        Client that asks for a clean task to add to its task list
        :return:
        """
        rospy.wait_for_service('give_zones')
        print("trying to request")
        try:
            zone_request = rospy.ServiceProxy('give_zones', RequestCleanTask)
            clean_task = zone_request(self.ID)
            clean_task = self.parse_task(clean_task)
            self.Tasks.put(clean_task.priority, clean_task)
            rospy.loginfo(self.Tasks)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)




    #When service is requested by the basebot, updates both the avoid status to true and the coordinate
    def update_avoid_status_handler(self, req):
        """
        Service that adds an avoid Task to the task list
        :param req: parameters for an avoid Task
        :return:
        """
        rospy.loginfo("Hamdlimg avoid status request")
        rospy.loginfo(self.Tasks)
        avoid_task = self.parse_task(req)
        self.Tasks.put(avoid_task.priority, avoid_task)
        rospy.loginfo(self.Tasks)
    #TODO make the service to update the avoid status and avoid coordinate from Task



    #When service is requested, updates amd adds a dump task to the priority queue
    def update_dump_status(self):
        return



    #Send task to be identified for appropriate execution
    def send_to_id(self):
        """
        Sends the first element in the priority queue to ve identified
        :return:
        """
        self.currentTask = self.Tasks.get()
        self.TaskIdentifier.task_ID(self.currentTask)



if __name__ == "__main__":
    ts = TaskSeeker()