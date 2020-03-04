#!/usr/bin/python
# title           :Encoder.py
# description     :node for reading and publishing positional data for smallbot
# author          :Sean Tidd
# date            :2020-02-11
# version         :0.1
# notes           :
# python_version  :3.5
# ==============================================================================
import RPi.GPIO as GPIO
import rospy
import math
from support.Constants import *
from geometry_msgs.msg import Pose
from navigation.msg import IMU_msg, direct_msg


#TODO make a listener for imu angle and make a ros publisher
class Encoder:
    def __init__(self):
        rospy.init_node("Encoder", anonymous=True)
        self.ticks = 0.0
        self.xDist = 0.0
        self.yDist = 0.0
        self.angle = 0.0
        self.oldDist = 0.0
        self.direct = 1
        self.isPaused = False
        angle_listener = rospy.Subscriber("/IMU",IMU_msg, self.angle_callback)
        direction_listener = rospy.Subscriber("/drive_direct",direct_msg, self.drive_direct_callback)
        #isPaused_listener = rospy.Subscriber("/isPaused",Pause_msg, self.pause_callback)
        self.pub = rospy.Publisher("odom",Pose,queue_size=10)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENCODER1_PIN1, GPIO.IN)
        GPIO.setup(ENCODER1_PIN2, GPIO.IN)
        GPIO.add_event_detect(ENCODER1_PIN1, GPIO.RISING, callback=self.encoder_callback1, bouncetime=300)
        GPIO.add_event_detect(ENCODER1_PIN2, GPIO.RISING, callback=self.encoder_callback2, bouncetime=300)



    def encoder_callback1(self, channel):
        self.ticks += 0.5

    def encoder_callback2(self, channel):
        self.ticks += 0.5

    def angle_callback(self, msg):
        self.angle = msg.zRotation

    def drive_direct_callback(self,msg):
        self.direct = msg.direct

    #def pause_callback(self,msg):
    #    self.isPaused = msg
    #    return

    def convert_to_distance(self):
        """
        Converts positional data from sensors to update smallbot location
        :return:
        """
        total_revs = self.ticks
        raw_dist = total_revs * TREAD_CIRCUMFERENCE
       # print("Dist: ",raw_dist)
        self.yDist += (raw_dist-self.oldDist) * math.sin(self.angle) * self.direct
        self.xDist += (raw_dist-self.oldDist) * math.cos(self.angle) * self.direct
        self.oldDist = raw_dist

    def pub_dist(self):
        """
        Publishes the updated position of the smallbot
        :return:
        """
        self.convert_to_distance()
        msg = Pose()
        msg.position.x = self.xDist
        msg.position.y = self.yDist
        msg.orientation.z = self.angle
        print(msg.position)
        self.pub.publish(msg)

if __name__ == "__main__":

 encoder = Encoder()
 while not rospy.is_shutdown():
     try:
         if encoder.isPaused is False:
             encoder.pub_dist()
     except KeyboardInterrupt:
        GPIO.cleanup()
        break
