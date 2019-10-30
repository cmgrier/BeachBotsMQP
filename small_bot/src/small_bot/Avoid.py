"""from smallBot.NavToGoal import NavToGoal
"""
import rospy

#This class combines all necessary classes for performing the Avoid task

class Avoid:
    def __init__(self):
        """self.NavToGoal = NavToGoal()
        """
    #Makes small robot navigate to the avoid coordinate
    def handle_avoid(self, task):
        """self.NavToGoal.navigate(task.Avoid.coord)
        """
        # TODO delete bottom code and uncomment when done
        rospy.loginfo("In Avoid")