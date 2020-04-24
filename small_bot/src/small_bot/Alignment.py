#!/usr/bin/env python

# Imports
import rospy
import maestro
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
        self.servo = maestro.Controller()

        # Subscribers
        rospy.Subscriber('near_centroid', Point, self.centroid_callback)
        rospy.Subscriber('large_box', Point, self.box_callback)
        rospy.Subscriber('pickup_done', Bool, self.pickup_complete)
        self.yaw_pub = rospy.Publisher('cam_yaw', Twist, queue_size=10)
        self.pickup_ready = rospy.Publisher('pickup_flag', Bool, queue_size=10)

        # Configure the Camera Servo
        self.cam_servo_pin = CAMERA
        self.position = 4500  # DO NOT FORGET TO CHANGE THIS BASED ON CVOUTPUT
        self.h = 480
        self.w = 500
        self.area = 0

        self.threshold = 15
        self.twitch = 30
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

            if 5000.0 > self.position > 4000.0:
                self.servo.setTarget(self.cam_servo_pin, self.position)
        else:
            if 5000.0 > self.position > 4000.0:
                self.servo.setTarget(self.cam_servo_pin, self.position)

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
                turn_angle = -.2
            elif centroid[0] < video_centroid[0] - yaw_thresh:
                turn_angle = .2
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

        if 0 < self.area < 30000:
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
            rospy.sleep(5)
            for i in range(300):
                msg = Twist()
                msg.linear.x = .7
                msg.linear.y = 0
                msg.linear.z = 0

                msg.angular.x = 0
                msg.angular.y = 0
                msg.angular.z = 0
                self.yaw_pub.publish(msg)
            # rospy.sleep(2)
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0

            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            self.yaw_pub.publish(msg)

            print("Done driving forward, picking up can")
            rospy.sleep(5)

            msg = Bool()
            msg.data = True
            self.pickup_ready.publish(msg)
            self.pickup_done = False

            while not self.pickup_done:
                rospy.sleep(1)

    def cleanup(self):
        """
        Cleanup
        :return: void
        """
        self.servo.close()


if __name__ == "__main__":
    align_node = Alignment()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        align_node.cleanup()
