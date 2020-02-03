#!/usr/bin/python
from math import *
from support.Constants import *
import numpy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class Kinematics:

    def fwkin(self, ang0, ang1, graph=False):
        """ Takes the angles of the joints in the arm, ang0 and ang1 referring to the
        shoulder and elbow joints respectively, and calculates the position of the
        end effector as a 3x1 matrix
        """
        ang0 = radians(ang0)
        ang1 = radians(ang1)
        matrix = self.transformMatrix(ang0+THETA1, ALPHA1, A1, D1)
        matrix2 = numpy.dot(self.transformMatrix(ang0 + THETA1, ALPHA1, A1, D1), self.transformMatrix(ang1 + THETA2, ALPHA2, A2, D2))
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
        return numpy.dot(self.transformMatrix(ang0+THETA1, ALPHA1, A1, D1), self.transformMatrix(ang1+THETA2, ALPHA2, A2, D2))

    def getEndEffectorFromAngles(self, ang0, ang1, graph=False ):
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

    def transformMatrix(self, theta, alpha, a, d):
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
        :param x: end effector x-coord
        :param y: end effector y-coord
        :return: joint angles to result in the end effector reaching the desired position
        """

        # When using invkin() put it in a try-catch for ValueError to handle the edge case of when
        # the end effector coordinates are out of the arm's task space

        x = x+A1+A2     # transform coordinates from end effector to shoulder origin
        D3 = sqrt((y*y)+(x*x))
        d3 = acos(round(((A1*A1)+(A2*A2)-(D3*D3))/(2*A1*A2),3))
        theta2 = 180 - degrees(d3)
        theta1 = 90 - degrees(acos((y/D3))) - degrees(acos(round(((A1*A1)+(D3*D3)-(A2*A2))/(2*A1*D3),3)))
        if x < 0:
            theta1 = -90 - degrees(acos((y/D3))) - degrees(acos(round(((A1*A1)+(D3*D3)-(A2*A2))/(2*A1*D3),3)))
        return (theta1,theta2)

if __name__=="__main__":
    k = Kinematics()
    k.getEndEffectorFromAngles(k.invkin(-0.5,-0.2)[0], k.invkin(-0.5,-0.2)[1])