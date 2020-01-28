#!/usr/bin/python
from math import *
from support.Constants import *
import numpy


# Collector Arm D-H Frame Data
A1 = 0
ALPHA1 = pi/2.0
D1 = 0
THETA1 = 0
A2 = 0
ALPHA2 = 0
D2 = 0.180  # meters
THETA2 = pi/2.0


class Kinematics:


    def __init__(self, smallbot):
        self.smallbot = smallbot

    def fwkin(self, ang0, ang1):
        """ Takes the angles of the joints in the arm, ang0 and ang1 referring to the
        shoulder and elbow joints respectively, and calculates the position of the
        end effector as a 3x1 matrix
        """
        ang0 = ang0*pi/180.0
        ang1 = ang1*pi/180.0
        return numpy.dot(self.transformMatrix(ang0+THETA1, ALPHA1, A1, D1), self.transformMatrix(ang1+THETA2, ALPHA2, A2, D2))

    def getEndEffectorFromAngles(self,ang0,ang1):
        """
        Gets the end effector's final position after the joints rotate to a certain angle
        :param ang0: joint 0 angle
        :param ang1: joint 1 angle
        :return: [x,y,z] position in meters
        """
        matrix = self.fwkin(ang0, ang1)
        print(matrix)
        return [matrix[0][3], matrix[1][3], matrix[2][3]]

    def transformMatrix(self, theta, alpha, a, d):
        T = [
                [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                [sin(theta), cos(alpha)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]
             ]
        return T


if __name__=="__main__":
    k = Kinematics(2)
    print(k.getEndEffectorFromAngles(90, 90))