class Director:
    def __init__(self):
        pass

    # The following methods will create Tasks to send to the small bots that will move them
    def go_back_to_zone(self, workerID):
        pass

    # sends a Task to robot with given ID to divert off current path in order to avoid another robot
    def avoid_to_direction(self, direction, workerID):
        pass

    # sends a Task to a robot to not move in order for another robot to avoid it
    # only called when all directions are unsafe to avoid to
    def play_possum(self, workerID):
        pass