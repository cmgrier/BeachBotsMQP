"""from smallBot.NavToGoal import NavToGoal
from smallBot.TrashBin import TrashBin
"""
import rospy

#This class combines all necessary classes for performing the Dump task

class Dump:
    def __init__(self):
        """self.NavToGoal = NavToGoal()
        self.TrashBin = TrashBin()
        """

    #Makes small robot return to base bot and dump trash
    def handle_dump(self, task):
        """self.NavToGoal.navigate(task.Dump.coord) #Go to base bot dump coordinates
        self.removeTrash()
        """
        # TODO delete bottom code and uncomment when done
        rospy.loginfo("In Dump")

    #Lines up with base bot dumping mechanism and removes trash into it
    def removeTrash(self):
        #TODO add remove_from_bin() function once trash removal ideas have been made concrete
        return