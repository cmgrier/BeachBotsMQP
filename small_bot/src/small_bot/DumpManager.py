from data.Task import Task


class DumpManager:

    def __init__(self, smallbot, taskManager):
        self.smallbot = smallbot
        self.taskManager = taskManager
        pass

    def do_task(self, task):
        # this will attempt to complete the given dump task,
        # update progress on task and return the updated task
        return task
