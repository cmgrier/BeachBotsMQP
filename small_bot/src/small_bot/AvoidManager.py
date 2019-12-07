from data.Task import Task
from support.Constants import *
import geometry_msgs.msg


class AvoidManager:

    def __init__(self, smallbot, taskManager):
        self.smallbot = smallbot
        self.taskManager = taskManager
        self.counter = 0
        pass

    def do_task(self, task):
        # this will attempt to complete the given avoid task,
        # update progress on task and return the updated task
        self.counter = self.counter + 1
        twist = geometry_msgs.msg.Twist()
        twist.angular.z = 40
        self.taskManager.pub_vel(twist)
        if DEBUG:
            print("working on avoid task")
            print(self.counter)
        if self.counter > 100:
            print("AVOID TASK COMPLETE")
            self.counter = 0
            task.isComplete = True
        return task

