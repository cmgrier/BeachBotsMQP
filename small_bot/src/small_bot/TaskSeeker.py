#!/usr/bin/env python
import rospy
from data.Task import Task
from small_bot.srv import RequestCleanTask, PassAvoidTask, RequestDumpTask, Identify
#import RPi.GPIO as GPIO
from support.Constants import *


"""
This class resquest a new task and sends that task off to be identified.
This class also updates the status for the avoid and return tasks
"""
class TaskSeeker:


    def __init__(self, smallbot):
        rospy.init_node('task_seeker', anonymous=True)
        self.smallbot = smallbot
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(INTERRUPT_OUTPUT, GPIO.OUT)
        GPIO.output(INTERRUPT_OUTPUT, GPIO.LOW)
        """


    def parse_task(self, taskResponse):
        """
        Takes a service response and combines all the fields into a Task
        :param taskResponse: response received from a service
        :return: a Task
        """
        task = Task(taskResponse.zone, taskResponse.type)
        task.isActive = taskResponse.isActive
        task.isComplete = taskResponse.isComplete
        task.workerID = taskResponse.workerID
        task.start_point = taskResponse.startingPoint
        return task


    def request_ID(self):
        """
        Client that asks for a robot ID from basebot
        :return:
        """
        rospy.wait_for_service('identify_worker')
        print("trying to request")
        try:
            rospy.loginfo("Current ID: " + str(self.smallbot.id))
            request = rospy.ServiceProxy('identify_worker', Identify)
            service_result = request(self.smallbot.id)
            self.smallbot.id = service_result.newID
            rospy.loginfo("new ID: " + str(self.smallbot.id))
            topic_name = "robot_avoid_"+str(self.smallbot.id)
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
            clean_task = zone_request(self.smallbot.id)
            clean_task = self.parse_task(clean_task)
            self.smallbot.tasks.put(clean_task.priority, clean_task)
            rospy.loginfo(self.smallbot.tasks)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)




    #When service is requested by the basebot, updates both the avoid status to true and the coordinate
    def update_avoid_status_handler(self, req):
        """
        Service that adds an avoid Task to the task list
        :param req: parameters for an avoid Task
        :return: a string to notify service completion
        """
        rospy.loginfo("Handling avoid status request")
        avoid_task = self.parse_task(req)
        self.smallbot.tasks.put(avoid_task.priority, avoid_task)
        """
        GPIO.output(INTERRUPT_OUTPUT, GPIO.HIGH)
        GPIO.output(INTERRUPT_OUTPUT, GPIO.LOW)
        """
        return "Avoid Task Added To Robot ID: " + str(self.smallbot.id)




    #When service is requested, updates amd adds a dump task to the priority queue
    def request_dump_task(self):
        """
        Client that asks for a dump Task and adds it to the task list
        :return:
        """
        rospy.wait_for_service('give_dump')
        print("trying to request")
        try:
            zone_request = rospy.ServiceProxy('give_dump', RequestDumpTask)
            dump_task = zone_request(self.smallbot.id)
            dump_task = self.parse_task(dump_task)
            self.smallbot.tasks.put(dump_task.priority, dump_task)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)


