"""from smallBot.DrivePattern import DrivePattern
from smallBot.FindTrash import FindTrash
from smallBot.Collect import Collect
from smallBot.NavToGoal import NavToGoal
from smallBot.TaskStatus import TaskStatus
"""
import rospy
import RPi.GPIO as GPIO
import sys
from support.Constants import *

#This class combines and uses all necessary classes that will execute the clean zone task
#This includes moving in a pattern within a zone, locating trash, and collecting trash

class CleanZone:
    def __init__(self):
        GPIO.setmode(BCM)
        GPIO.setup(INTERRUPT_INPUT, GPIO.INPUT)
        GPIO.add_event_detect(INTERRUPT_INPUT, GPIO.FALLING, callback=this.task_added_interrupt, bouncetime=300)
        """self.DrivePattern = DrivePattern()
        self.FindTrash = FindTrash()
        self.Collect = Collect()
        self.NavToGoal = NavToGoal()
        self.TaskStatus = TaskStatus()
"""
    def task_added_interrupt(self, channel):
        print("Interrupt exiting")
        sys.exit(0)

    def handle_clean(self, task):
        while True:
            rospy.loginfo("In Clean Zone")