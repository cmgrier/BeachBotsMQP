from small_bot.CleanZone import CleanZone
from small_bot.Avoid import Avoid
from small_bot.Dump import Dump
from data.Task import Task


#This class identifies what type of task needs to be executed and sends it to the proper functions

class TaskIdentifier:


        #TS is TaskSeeker
    def __init__(self, TS):
        self.CleanZone = CleanZone()
        self.Avoid = Avoid()
        self.Dump = Dump()
        self.TS = TS

    #Sends the task to the proper class to be executed
    def task_ID(self, task):
        """
        Takes a task and sends it off to the proper method handler
        :param task: the task to be identified and executed
        :return:
        """

        if task.type == "avoid":
            self.Avoid.handle_avoid(task)
        elif task.type == "dump":
            self.Dump.handle_dump(task)
        else:
            self.CleanZone.handle_clean(task)

        #If task was not completed
        if task.isComplete is False:
            self.TS.Tasks.put(task.priority, task)
