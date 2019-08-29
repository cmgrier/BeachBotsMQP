# this class is a data class for the robot manager
class Robot:
    def __init__(self, id):
        self.workerID = id
        self.isBusy = False