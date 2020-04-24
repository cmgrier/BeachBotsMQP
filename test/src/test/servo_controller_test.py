#!/usr/bin/env python
import maestro
import time


class ServoController:
    def __init__(self):
        print("Initialized")
        servo = maestro.Controller()
        print("Made Controller")
        servo.setAccel(1, 4)  # set servo 0 acceleration to 4
        servo.setSpeed(1, 15)  # set speed of servo
        # print(servo.getMax(1))
        # print(servo.getMin(1))
        # servo.setRange(1, 0, 12000)
        # print(servo.getMax(1))
        # print(servo.getMin(1))
        servo.setTarget(0, 5500)  # set servo to move to center position
        time.sleep(5)
        servo.setTarget(0, 4500)
        time.sleep(5)
        # x = servo.getPosition(1)  # get the current position of servo 1
        print("Done")
        servo.close()


if __name__ == "__main__":
    sc = ServoController()
