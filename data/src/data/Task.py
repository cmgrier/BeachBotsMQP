# data of the different tasks to be completed by the small bots
class Task:
    def __init__(self, zone=None, type=None):
        self.isActive = False
        self.isComplete = False
        self.workerID = -1
        self.type = type or "clean"
        self.zone = zone
        self.priority = 3
        self.start_point = None
        self.set_priority()

#set the priority level of the task based on the type
    def set_priority(self):
        if self.type == "clean":
            self.priority = 3
        elif self.type == "avoid":
            self.priority = 1
        elif self.type == "dump":
            self.priority = 2
        else:
            self.priority = 4

    def __repr__(self):
        return str((self.type, self.workerID, self.zone, self.isActive, self.isComplete))


    def __str__(self):
        return str((self.type, self.workerID, self.zone, self.isActive, self.isComplete))

    def to_service_format(self):
        return [self.isActive, self.isComplete, self.workerID, self.type, self.zone, self.start_point]

    def make_safe_task(self, worker_id):
        self.isActive = None
        self.isComplete = None
        self.type = "safe"
        self.zone = None
        self.priority = None
        self.start_point = None
        self.workerID = worker_id

    def make_avoid_task(self, goal, worker_id):
        self.isComplete = False
        self.isActive = False
        self.type = "avoid"
        self.set_priority()
        self.zone = None
        self.workerID = worker_id
        self.start_point = goal