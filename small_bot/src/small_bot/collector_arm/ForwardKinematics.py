from support.src.support.Constants import *
from math import *


class ForwardKinematics:

    def __init__(self, smallbot):
        self.smallbot = smallbot

    def fwkin(self, ang0, ang1):
        """ Takes the angles of the joints in the arm, ang0 and ang1 referring to the
        shoulder and elbow joints respectively, and calculates the position of the
        end effector as a 3x1 matrix
        """
        

    def transformMatrix(self, theta, alpha, a, d):
        T = [
                [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                [sin(theta), cos(alpha)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]
             ]
        return T