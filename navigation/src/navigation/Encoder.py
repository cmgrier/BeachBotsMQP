#!/usr/bin/python

import RPi.GPIO as GPIO
import rospy
import math
from support.Constants import *
from geometry_msgs.msg import Pose
from navigation.msg import IMU_msg

#TODO make a listener for imu angle and make a ros publisher
class Encoder:
    def __init__(self):
        rospy.init_node("Encoder", anonymous=True)
        self.ticks = 0.0
        self.xDist = 0.0
        self.yDist = 0.0
        self.angle = 0.0
        self.isPaused = False
        angle_listener = rospy.Subscriber("/IMU",IMU_msg, self.angle_callback)
        self.pub = rospy.Publisher("odom",Pose,queue_size=10)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENCODER1_PIN1, GPIO.IN) #TODO add this constant
        GPIO.setup(ENCODER1_PIN2, GPIO.IN) #TODO add this constant
        GPIO.add_event_detect(ENCODER1_PIN1, GPIO.RISING, callback=self.encoder_callback1, bouncetime=300)
        GPIO.add_event_detect(ENCODER1_PIN2, GPIO.RISING, callback=self.encoder_callback2, bouncetime=300)



    def encoder_callback1(self, channel):
        self.ticks += 0.5

    def encoder_callback2(self, channel):
        self.ticks += 0.5

    def angle_callback(self, msg):
        self.angle = msg.zRotation

    def clearTicks(self): #TODO make this a ros serice handler
        self.ticks = 0.0


    def convertToDistance(self):
        totalRevs = self.ticks
        rawDist = totalRevs * TREAD_CIRCUMFERENCE  #TODO add this constant
        print("Dist: ",rawDist)
        #self.yDist += rawDist * math.sin(self.angle)
        #self.xDist += rawDist * math.cos(self.angle)

    def pubDist(self):
        self.convertToDistance()
        msg = Pose
        msg.position.x = self.xDist
        msg.position.y = self.yDist
        msg.orientation.z = self.angle
        self.pub.publish(msg)

if __name__ == "__main__":

 encoder = Encoder()
 while not rospy.is_shutdown():
     try:
         encoder.convertToDistance()
     except KeyboardInterrupt:
        GPIO.cleanup()
        break