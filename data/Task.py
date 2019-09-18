# data of the different tasks to be completed by the small bots
class Task:
    def __init__(self, zone):
        self.isActive = False
        self.isComplete = False
        self.workerID = -1
        self.zone = zone
        self.robot_to_avoid = None
