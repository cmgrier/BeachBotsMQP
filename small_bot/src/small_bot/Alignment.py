#!/usr/bin/env python

# Imports
import rospy
import pigpio
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
from support.Constants import *


class Alignment:
    def __init__(self):
        """
        Initializations
        """

        # Initialization of the node
        rospy.init_node('Alignment')

        # Subscribers
        rospy.Subscriber('near_centroid', Point, self.centroid_callback)
        rospy.Subscriber('large_box', Point, self.box_callback)
        rospy.Subscriber('pickup_done', Bool, self.pickup_complete)
        self.yaw_pub = rospy.Publisher('cam_yaw', Twist, queue_size=10)
        self.pickup_ready = rospy.Publisher('pickup_flag', Bool, queue_size=10)

        # Connect to local Pi.
        self.pi = pigpio.pi()

        # Configure the Camera Servo
        self.cam_servo_pin = SERVO_CAM
        self.position = 700  # DO NOT FORGET TO CHANGE THIS BASED ON CVOUTPUT
        self.h = 480
        self.w = 500
        self.area = 0

        self.threshold = 20
        self.twitch = 50
        self.stopped_flag = False
        self.pickup_done = True

    def pickup_complete(self, msg):
        """
        Pickup Done Callback
        :param msg:
        :return: void
        """
        self.pickup_done = True

    def box_callback(self, msg):
        """
        Bounding box callback
        :param msg: BoundingBox2D
        :return: void
        """
        self.area = msg.x

    def centroid_callback(self, msg):
        """
        Callback for the centroid subscriber
        :param msg: Point
        :return: void
        """

        # Centroid from message
        centroid = (msg.x, msg.y)

        # Math for moving the camera
        video_centroid = (self.w / 2, self.h / 2 - 20)
        if centroid[0] > 0 and centroid[1] > 0:

            if centroid[1] > video_centroid[1] + self.threshold:
                self.position -= self.twitch
            elif centroid[1] < video_centroid[1] - self.threshold:
                self.position += self.twitch

            if 1100.0 > self.position > 600.0:
                self.pi.set_servo_pulsewidth(self.cam_servo_pin, self.position)
                rospy.sleep(0.5)
        else:
            if 1100.0 > self.position > 600.0:
                self.pi.set_servo_pulsewidth(self.cam_servo_pin, self.position)

        move = self.yaw_alignment(centroid, video_centroid)
        if move:
            self.drive_forward()

    def yaw_alignment(self, centroid, video_centroid, yaw_thresh=50):
        """
        Moves the motors until we are inline with the can
        :param centroid: the centroid tuple
        :param video_centroid: the video streams centroid
        :param yaw_thresh: the yaw threshold
        :return: move_trigger
        """
        turn_angle = 0
        move_trigger = False

        if centroid[0] > 0 and centroid[1] > 0:
            if centroid[0] > video_centroid[0] + yaw_thresh:
                turn_angle = -.35
            elif centroid[0] < video_centroid[0] - yaw_thresh:
                turn_angle = .35
            else:
                move_trigger = True

            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0

            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = turn_angle
        else:
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0

            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0

        self.yaw_pub.publish(msg)
        return move_trigger

    def drive_forward(self):
        """
        Drive the robot forward a small amount
        :return: void
        """

        print(self.area)

        if self.area < 30000:
            print("Driving Forward +++++++++++++++++++++")
            msg = Twist()
            msg.linear.x = .7
            msg.linear.y = 0
            msg.linear.z = 0

            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            self.yaw_pub.publish(msg)
            rospy.sleep(.2)
            self.stopped_flag = False

        else:
            print("Stopped -----------------------------")
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0

            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            self.yaw_pub.publish(msg)
            self.stopped_flag = True

        if self.area > 30000 and self.stopped_flag and self.pickup_done:

            # Drive Forward
            print("Found Can, Driving forward")
            msg = Bool()
            msg.data = True
            self.pickup_ready.publish(msg)
            self.pickup_done = False
            for i in range(120):
                msg = Twist()
                msg.linear.x = .7
                msg.linear.y = 0
                msg.linear.z = 0

                msg.angular.x = 0
                msg.angular.y = 0
                msg.angular.z = 0
                self.yaw_pub.publish(msg)

            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0

            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            self.yaw_pub.publish(msg)
            while not self.pickup_done:
                rospy.sleep(1)

    def cleanup(self):
        """
        Cleanup
        :return: void
        """
        self.pi.set_servo_pulsewidth(self.cam_servo_pin, 0)
        self.pi.stop()


if __name__ == "__main__":
    align_node = Alignment()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        align_node.cleanup()
