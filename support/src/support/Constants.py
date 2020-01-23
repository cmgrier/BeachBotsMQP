#!/usr/bin/env python

TRASH_BIN_THRESHOLD = 12  # TODO: Get the actual value for this constant

STORED_MAP_SIZE = 10
SAFE_DISTANCE_BETWEEN_BOTS = 100
AVOID_DISTANCE = 30

ZONE_WIDTH = 20
LANDING_STRIP_WIDTH = 20

# ask Chris. Essentially we prioritize to avoiding robots by backing up first rather than trying to move forward
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
TREAD_CIRCUMFERENCE = 0.151 # mm

# Camera Servo
SERVO_CAM = 17

# make true to activate debug print statements
DEBUG = False

# Collector Arm D-H Frame Data
A1 = 0
ALPHA1 = 0
D1 = 0
THETA1 = 0
A2 = 0
ALPHA2 = 0
D2 = 0
THETA2 = 0
