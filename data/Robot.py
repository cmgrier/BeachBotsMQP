# this class is a data class for the robot manager
class Robot:
    def __init__(self, id):
        self.workerID = id
        self.pose = [[0, 0, 0], [0, 0, 0, 0]] # this will be a ros pose data type
        self.isBusy = False
        self.task = None
