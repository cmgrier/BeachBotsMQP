#!/usr/bin/env python

TRASH_BIN_THRESHOLD = 12  # TODO: Get the actual value for this constant

STORED_MAP_SIZE = 10
SAFE_DISTANCE_BETWEEN_BOTS = 100
AVOID_DISTANCE = 30

ZONE_LENGTH = 2
ZONE_WIDTH = .5
LANDING_STRIP_WIDTH = .25

# Prioritize to avoiding robots by backing up first rather than trying to move forward
AVOID_DIRECTION_PRIORITY_LIST = [3, 4, 2, 5, 1, 6, 0, 7]

# AStar
STANDARD_MOVE_COST = 1
DIFFICULT_TERRAIN_FACTOR = 1
TURNING_FACTOR = 1

TERRAIN_TOO_DIFFICULT = 30
ZONE_TOO_DIFFICULT_PERCENT = .1

# Small Motors
SMALL_L_WHEEL_PIN = 12
SMALL_R_WHEEL_PIN = 13

SMALL_L_DIRECT_1 = 4
SMALL_L_DIRECT_2 = 27

SMALL_R_DIRECT_1 = 21
SMALL_R_DIRECT_2 = 26

# Small Robot Encoder Interrupts
ENCODER1_PIN1 = 6
ENCODER1_PIN2 = 16
TREAD_CIRCUMFERENCE = 0.151 # meters

# Camera Servo
SERVO_CAM = 17

# ZED
MESH_REFRESH_RATE = 2   # update mesh per second
# this is the z value of the triangle's normal to indicate if the triangle is traversable
TRIANGLE_NORMAL_Z_TRAVERSABLE = .7
TRIANGLE_NORMAL_ANGLE_TRAVERSABLE = .523599
MAX_INCLINE = 30    # max angle incline
# visible area for zed (used for finding visible zones)
Y_MIN = .5
Y_MAX = 5.0
X_MAX = 2.25
# Occupancy grid width
OG_WIDTH = 100

# Navigation
DISTANCE_THRESHOLD_MIN = 0.2
DISTANCE_THRESHOLD_MAX = 0.2
ANGLE_THRESHOLD_MIN = 1.0
ANGLE_THRESHOLD_MAX = 1.0

#PID
TURN_ANGLE_KP = 1.0
TURN_ANGLE_IP = 0.1
TURN_ANGLE_DP = 0.01
DRIVE_DIST_KP = 1.0
DRIVE_DIST_IP = 0.1
DRIVE_DIST_DP = 0.01

# make true to activate debug print statements
DEBUG = False


# Collector Arm D-H Frame Data
A1 = 0.130  # meters
ALPHA1 = 0.0
D1 = 0.0
THETA1 = 0.0
A2 = 0.180  # meters
ALPHA2 = 0.0
D2 = 0.0
THETA2 = 0.0

# Collector Arm Motors
STEP_ANGLE = 0.035  # Degrees per step
DIR = 0  # Direction pin
STEP = 0  # Step pin
SWITCH = 9  # Miso limit switch
CW = 1
CCW = 0

COIL_A_1_PIN = 18
COIL_A_2_PIN = 19
COIL_B_1_PIN = 20
COIL_B_2_PIN = 24

JOINT1_SERVO = 7  # (CE1)
JOINT1_START = 6

GRIPPER_SERVO = 8  # (CE0)
GRIPPER_OPEN = 2  # Duty Cycle
GRIPPER_CLOSE = 10  # Duty Cycle


