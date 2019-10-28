from smallBot.CleanZone import CleanZone
from smallBot.Avoid import Avoid
from smallBot.Dump import Dump
from data.Task import Task


#This class identifies what type of task needs to be executed and sends it to the proper functions

class TaskIdentifier:

    def __init__(self):
        self.CleanZone = CleanZone()
        self.Avoid = Avoid()
        self.Dump = Dump()


    #Sends the task to the proper class to be executed
    def task_ID(self, task):

        if task.type == "avoid":
            self.Avoid.handle_avoid(task)
        elif task.type == "dump":
            self.Dump.handle_dump(task)
        else:
            self.CleanZone.handle_clean(task)


if __name__=='__main__':
    ti = TaskIdentifier()
    ti.task_ID(Task(3))
