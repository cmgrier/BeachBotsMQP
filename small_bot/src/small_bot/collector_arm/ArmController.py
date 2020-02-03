from small_bot.Kinematics import Kinematics


class ArmController:

    def __init__(self,pin0,pin1,pin2):
        """
        Constructor for ArmController class
        :param pin0: pin for joint 0 stepper motor
        :param pin1: pin for joint 1 servo
        :param pin2: pin for gripper servo
        """
        self.pin0 = pin0
        self.pin1 = pin1
        self.pin2 = pin2
        self.kin = Kinematics()

    def move_end_effector(self,x,y):
        """
        Move the end effector to the xy-coordinates
        :param x: final x coordinate with respect to the end effector in meters
        :param y: final y coordinate with respect to the end effector in meters
        :return:
        """
        #TODO
        angles = self.kin.invkin(x,y)
        self.turn_joint0(angles[0])
        self.turn_joint1(angles[1])


    def turn_joint0(self,angle):
        """
        Turn the stepper motor for joint 0 to a specific angle
        :param angle: angle of the joint to turn to in degrees
        :return:
        """
        #TODO

    def turn_joint1(self,angle):
        """
        Turn the servo for joint 1 to a specific angle
        :param angle: angle of the joint to turn to in degrees
        :return:
        """
        #TODO

    def calibrate_joints(self):
        """
        Moves the motors until they are zeroed
        :return:
        """
        #TODO

    def close_gripper(self,status):
        """
        Moves the servo for the gripper
        :param status: True closes the gripper and False opens it
        :return:
        """
        #TODO