from data.Task import Task
from small_bot.CleanManager import CleanManager

class TaskManager:

    def __init__(self):
        self.cleanManager = CleanManager()
        self.dumpManager = DumpManager()
        self.avoidManager = AvoidManager()

    def do_task(self, task):
        if task.type == "clean":
            return self.cleanManager.do_task(task)
        elif task.type == "dump":
            return self.dumpManager.do_task(task)
        elif task.type == "avoid":
            return self.avoidManager.do_task(task)
        else:
            return task
