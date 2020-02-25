#!/usr/bin/env python
import rospy
from data.Task import Task
from data.Zone import Zone
from small_bot.srv import RequestCleanTask, RequestTask, RequestDumpTask, Identify, RequestOG
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
        zone = Zone(taskResponse.zoneCorners, taskResponse.zoneID)
        task = Task(zone, taskResponse.type)
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
        print("trying to request new ID")
        try:
            rospy.loginfo("Current ID: " + str(self.smallbot.id))
            request = rospy.ServiceProxy('identify_worker', Identify)
            service_result = request(self.smallbot.id)
            self.smallbot.id = service_result.newID
            rospy.loginfo("new ID: " + str(self.smallbot.id))

        except rospy.ServiceException, e:
            rospy.loginfo("Service call (request_ID) failed: %s" % e)


    def request_clean_task(self):
        """
        Client that asks for a clean task to add to its task list
        :return:
        """
        rospy.wait_for_service('give_zones')
        print("trying to request clean task for robot:")
        print(self.smallbot.id)
        try:
            zone_request = rospy.ServiceProxy('give_zones', RequestTask)
            if DEBUG:
                print("sending request")
            clean_task = zone_request(self.smallbot.id)
            if DEBUG:
                print(clean_task)
            clean_task = self.parse_task(clean_task)
            if DEBUG:
                print(clean_task)
            if clean_task.type == "safe":
                self.smallbot.isCleaning = False
            else:
                self.smallbot.tasks.put(clean_task.priority, clean_task)
            rospy.loginfo(self.smallbot.tasks)
        except rospy.ServiceException, e:
            print("error")
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
        if DEBUG:
            print("trying to request dump task")
        try:
            zone_request = rospy.ServiceProxy('give_dump', RequestTask)
            dump_task = zone_request(self.smallbot.id)
            dump_task = self.parse_task(dump_task)
            self.smallbot.tasks.put(dump_task.priority, dump_task)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)


    def request_avoid_status(self):
        rospy.wait_for_service('give_avoid_status')
        if DEBUG:
            print("trying to request avoid status")
        try:
            avoid_request = rospy.ServiceProxy('give_avoid_status', RequestTask)
            avoid_task = avoid_request(self.smallbot.id)
            avoid_task = self.parse_task(avoid_task)
            if DEBUG:
                print("avoid task:")
                print(avoid_task)
            if avoid_task.type == "avoid":
                if DEBUG:
                    print("task is avoid")
                if not self.smallbot.tasks.has(1):
                    print("adding avoid task")
                    self.smallbot.tasks.put(avoid_task.priority, avoid_task)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)

    def request_og(self):
        rospy.wait_for_service('give_og')
        if DEBUG:
            print("trying to request og")
        try:
            og_request = rospy.ServiceProxy('give_og', RequestOG)
            return_data = og_request(self.smallbot.id)
            self.smallbot.baseBotPose = return_data.baseBotPose
            self.smallbot.occupancyGrid = return_data.occupancyGrid
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)