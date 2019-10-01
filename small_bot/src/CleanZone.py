"""from smallBot.DrivePattern import DrivePattern
from smallBot.FindTrash import FindTrash
from smallBot.Collect import Collect
from smallBot.NavToGoal import NavToGoal
from smallBot.TaskStatus import TaskStatus
"""
#This class combines and uses all necessary classes that will execute the clean zone task
#This includes moving in a pattern within a zone, locating trash, and collecting trash

class CleanZone:
    def __init__(self):
        """self.DrivePattern = DrivePattern()
        self.FindTrash = FindTrash()
        self.Collect = Collect()
        self.NavToGoal = NavToGoal()
        self.TaskStatus = TaskStatus()
"""
    def handle_clean(self, task):
        """self.NavToGoal.navigate(task.coord) #Navigate to zone
        while(self.TaskStatus.is_full() is not True or task.isComplete is True):
            self.FindTrash.find_trash()
            #TODO if trash is found and in grabbing distance then Collect otherwise continue drive pattern
            self.DrivePattern.follow_pattern()
        if self.TaskStatus.is_full() is True:
            task.Dump.status = True"""

        #TODO delete bottom code and uncomment when done
        print("In Clean Zone")