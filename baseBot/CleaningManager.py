from baseBot.MapManager import MapManager
from baseBot.RobotManager import RobotManager

# this class will hold all of the other managers and relay information between them.
# It will also hold on to the queued tasks and send them to
from data.Task import Task


class CleaningManager:
    def __init__(self, robots):
        self.mapManager = MapManager()
        self.cleaningTasks = list()
        self.robotManager = RobotManager(robots)

        self.create_cleaning_tasks()

    # run on startup
    def create_cleaning_tasks(self):
        for zone in self.mapManager.zones:
            self.cleaningTasks.append(Task(zone))

    # checks if each robot is busy, if not, assigns a task
    def assign_available_robots(self):
        for robot in self.robotManager.managedRobots:
            if not robot.isBusy:
                self.assign_robot_task(robot)

    # assigns the next task to the given robot
    def assign_robot_task(self, robot):
        for task in self.cleaningTasks:
            if task.workerID == -1 and task.isActive is False and task.isComplete is False:
                task.workerID = robot.workerID
                task.isActive = True
                for managedRobot in self.robotManager.managedRobots:
                    if managedRobot.workerID == robot.workerID:
                        managedRobot.isBusy = True
                        managedRobot.task = task
                return
