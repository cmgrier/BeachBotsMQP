from data.Task import Task


class CleanManager:

    def __init__(self, smallbot):
        self.smallbot = smallbot
        pass

    def do_task(self, task):
        # this will attempt to complete the given clean task,
        # update progress on task and return the updated task
        return task
