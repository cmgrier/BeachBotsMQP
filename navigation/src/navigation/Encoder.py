import RPi.GPIO as GPIO
import rospy
import math
from support.Constants import *
from geometry_msgs.msg import Pose

#TODO make a listener for imu angle and make a ros publisher
class Encoder:
    def __init__(self):
        self.ticks = 0.0
        self.xDist = 0.0
        self.yDist = 0.0
        self.angle = 0.0
        self.isPaused = False
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENCODER_PIN1, GPIO.IN) #TODO add this constant
        GPIO.setup(ENCODER_PIN2, GPIO.IN) #TODO add this constant
        GPIO.add_event_detect(ENCODER_PIN1, GPIO.RISING, callback=self.encoder_callback, bouncetime=300)
        GPIO.add_event_detect(ENCODER_PIN2, GPIO.RISING, callback=self.encoder_callback, bouncetime=300)



    def encoder_callback(self):
        self.ticks += 0.5


    def clearTicks(self): #TODO make this a ros serice handler
        self.ticks = 0


    def convertToDistance(self):
        totalRevs = self.ticks
        rawDist = totalRevs * TREAD_CIRCUMFERENCE  #TODO add this constant
        self.yDist += rawDist * math.sin(self.angle)
        self.xDist += rawDist * math.cos(self.angle)

    def pubDist(self):
        self.convertToDistance()
        msg = Pose
        msg.position.x = self.xDist
        msg.position.y = self.yDist
        msg.orientation.z = self.angle
        #pub.publish(msg)