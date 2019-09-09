"""from smallBot.TrashBin import TrashBin
from smallBot.RobotLocation import RobotLocation
from support import Constants
"""
# This class deals with managing all the statuses that pertain to the task at hand
# The statuses include: is trash bin full, where is the small bot,...

class TaskStatus:
    def __init__(self):
        """self.TrashBin = TrashBin()
        self.RobotLocation = RobotLocation()
        self.TaskCompletion = TaskCompletion()
        """

    #@brief Reads the sensor in the robot's trash bin to determine if full
    #@param
    #@return boolean    Returns true if sensor reads meet threshold, otherwise false
    def is_full(self):
        """if TrashBin.readBin() > Constants.trashBinThreshold:
            return  True
        else:
            return False
        """
        return
    #@brief Compares the robot's location to that of the task's desired zone
    #@param      tasks       The task containing the zone to compare to
    #@return     boolean     Returns true if robot is within zone, otherwise false
    def within_zone(self,task):
        return


    #@brief Checks to see if the task goal has bee achieved
    #@param     task        The task containing the goal
    #@return    boolean     Returns true if goal has been met, otherwise false
    def is_completed(self,task):
        return
