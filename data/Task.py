# data of the different tasks to be completed by the small bots
class Task:
    def __init__(self, zone, type=None):
        self.isActive = False
        self.isComplete = False
        self.workerID = -1
        self.type = type or "clean"
        self.zone = zone
        self.priority = 1
        self.set_priority()

#set the priority level of the task based on the type
    def set_priority(self):
        if self.type is "clean":
            self.priority = 3
        elif self.type is "avoid":
            self.priority = 1
        elif self.type is "dump":
            self.priority = 2
