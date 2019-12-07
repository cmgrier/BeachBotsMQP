from data.Task import Task
from support.Constants import *
import geometry_msgs.msg

class CleanManager:

    def __init__(self, smallbot, taskManager):
        self.smallbot = smallbot
        self.taskManager = taskManager
        self.counter = 0    # here for testing
        pass

    def do_task(self, task):
        # this will attempt to complete the given clean task,
        # update progress on task and return the updated task
        self.counter = self.counter + 1
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 60
        self.taskManager.pub_vel(twist)

        if DEBUG:
            print("working on cleaning task")
            print(self.counter)
        if self.counter > 100:
            print("CLEANING TASK COMPLETE")
            self.counter = 0
            task.isComplete = True
        return task
