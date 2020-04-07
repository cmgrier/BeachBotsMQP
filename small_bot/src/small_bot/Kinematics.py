#!/usr/bin/python
# title           :Kinematics.py
# description     :kinematic functions for the smallbot arm
# author          :Sean Tidd
# date            :2020-02-11
# version         :0.1
# notes           :
# python_version  :3.5
# ==============================================================================
from math import *
from support.Constants import *
import numpy
import matplotlib.pyplot as plt

class Kinematics:

    def fwkin(self, ang0, ang1, graph=False):
        """
        Takes the joint angles from joint0 and joint1 of the arm (shoulder and elcow)
        to get the end effector final position coordinates
        :param ang0: shoulder angle in degrees
        :param ang1: elvow angle in degrees
        :param graph: set to True to see a 3D representation
        :return:
        """
        ang0 = radians(ang0)
        ang1 = radians(ang1)
        matrix = self.transform_matrix(ang0+THETA1, ALPHA1, A1, D1)
        matrix2 = numpy.dot(self.transform_matrix(ang0 + THETA1, ALPHA1, A1, D1), self.transform_matrix(ang1 + THETA2, ALPHA2, A2, D2))
        if graph == True:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot([0,matrix[0][3]], [0,matrix[1][3]], [0,matrix[2][3]],color='g',marker='o')
            ax.plot([matrix[0][3],matrix2[0][3]], [matrix[1][3], matrix2[1][3]], [matrix[2][3],matrix2[2][3]], color='b',marker='o')
            ax.set_xlabel('x-axis')
            ax.set_ylabel('y-axis')
            ax.set_zlabel('z-axis')
            ax.set(xlim=(-.5, .5), ylim=(-.5, .5),zlim=(-.5, .5))
            plt.show()
        return numpy.dot(self.transform_matrix(ang0+THETA1, ALPHA1, A1, D1), self.transform_matrix(ang1+THETA2, ALPHA2, A2, D2))

    def get_end_effector_from_angles(self, ang0, ang1, graph=False ):
        """
        Gets the end effector's final position after the joints rotate to a certain angle
        :param ang0: joint 0 angle
        :param ang1: joint 1 angle
        :param graph: True to show 3D graph
        :return: [x,y,z] position in meters
        """
        matrix = self.fwkin(ang0, ang1, graph)
        print(matrix)
        return [matrix[0][3], matrix[1][3], matrix[2][3]]

    def transform_matrix(self, theta, alpha, a, d):
        """
         A single transformation from one frame to another using DH parameters
        :param theta: angle along the normal
        :param alpha: angle along the axis of rotation (Z-axis)
        :param a:  distance along the axis of rotation
        :param d: distance along the normal
        :return: a 4x4 transformation matrix
        """
        T = [
                [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]
             ]
        return T

    def invkin(self,x,y):
        """
        Inverse kinematics of 2-deg of freedom arm
        :param x: end effector x-coord (in terms of end effector origin)
        :param y: end effector y-coord (in terms of end effector origin)
        :return: joint angles to result in the end effector reaching the desired position
        """

        # When using invkin() put it in a try-catch for ValueError to handle the edge case of when
        # the end effector coordinates are out of the arm's task space

        x = x+A1+A2     # transform coordinates from end effector to shoulder origin
        D3 = sqrt((y*y)+(x*x))
        d3 = acos(round(((A1*A1)+(A2*A2)-(D3*D3))/(2*A1*A2),3))
        theta2 = 180 - degrees(d3)
        a4 = atan2(y,x)
        a2 = acos(round(((A1*A1)+(D3*D3)-(A2*A2))/(2*A1*D3),3))
        theta1 = degrees(a4-a2)
        return (theta1,theta2)

if __name__=="__main__":
    k = Kinematics()
    k.get_end_effector_from_angles(k.invkin(-0.12,0.2)[0], k.invkin(-0.12,0.2)[1])