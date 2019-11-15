from support.EqualPriorityQueue import EqualPriorityQueue
from small_bot.TaskIdentifier import TaskIdentifier
from data.Task import Task

class SmallBotManager:


    def __init__(self):
        self.isCleaning = true
        self.tasks = EqualPriorityQueue()
        self.id = -1
        self.position = Pose()
        self.taskManager = TaskManager()

        self.request_id()
        if self.id != -1:
            self.main()

    def main(self):
        while isCleaning:
            self.get_info()
            task = self.get_top_task()
            updated_task = self.do_task(task)
            if not Task.isComplete:
                self.tasks.put(updated_task.priority, updated_task)

    def get_info(self):
        # check the services and update SmallBotManager's info
        return 1

    def get_top_task(self):
        # return the top priority task in the task queue
        return 1

    def do_task(self, task):
        return self.taskManager.do_task(task)

    def request_id(self):
        pass
