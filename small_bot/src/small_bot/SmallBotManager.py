from support.EqualPriorityQueue import EqualPriorityQueue
from small_bot.TaskManager import TaskManager
from data.Task import Task
from geometry_msgs import Pose

class SmallBotManager:


    def __init__(self):
        self.isCleaning = True
        self.tasks = EqualPriorityQueue()
        self.id = -1
        self.position = Pose()
        self.taskManager = TaskManager(self)
        self.TaskSeeker(self)
        self.request_id()
        if self.id != -1:
            self.main()

    def main(self):
        """
        Main loop for small robot
        :return:
        """
        while self.isCleaning:
            self.get_info()
            task = self.tasks.get()
            if task.type == "end":
                break
            updated_task = self.do_task(task)
            if not task.isComplete:
                self.tasks.put(updated_task.priority, updated_task)

    def get_info(self):
        """
        Gathers info from any ros services or other communication
        :return:
        """
        # check the services and update SmallBotManager's info
        clean = 3
        if self.tasks.has(clean) == False:
            self.TaskSeeker.request_clean_task()

        #TODO update robots position as well as add more info
        #Maybe have a ros listener for current position
        #If dump satus is true then request dump task



    def do_task(self, task):
        """
        Sends a task to be identified and executed
        :param task: The task to be executed
        :return: An updated Task based on where it was left off
        """
        return self.taskManager.do_task(task)



    def request_id(self):
        """
        Requests basebot to ID smallbot
        :return:
        """
        self.TaskSeeker.request_ID()
        pass



if __name__ == "__main__":
    smallbot = SmallBotManager()
    smallbot.main()