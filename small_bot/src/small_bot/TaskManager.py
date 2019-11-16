from data.Task import Task
from small_bot.CleanManager import CleanManager
from small_bot.AvoidManager import AvoidManager
from small_bot.DumpManager import DumpManager

class TaskManager:

    def __init__(self, smallbot):
        self.cleanManager = CleanManager(smallbot)
        self.dumpManager = DumpManager(smallbot)
        self.avoidManager = AvoidManager(smallbot)

    def do_task(self, task):
        """
        ID the Task and execute it based on type
        :param task: The Task to be executed
        :return: Updated Task of where it left off on execution
        """
        if task.type == "clean":
            return self.cleanManager.do_task(task)
        elif task.type == "dump":
            return self.dumpManager.do_task(task)
        elif task.type == "avoid":
            return self.avoidManager.do_task(task)
        else:
            return task
