#!/usr/bin/env python
import maestro


class ServoController:
    def __init__(self):
        print("Initialized")
        servo = maestro.Controller('/dev/ttyAMA0')
        print("Made Controller")
        servo.setAccel(0, 4)  # set servo 0 acceleration to 4
        servo.setSpeed(0, 10)  # set speed of servo
        servo.setTarget(0, 2000)  # set servo to move to center position
        # x = servo.getPosition(1)  # get the current position of servo 1
        print("Done")
        servo.close()


if __name__ == "__main__":
    sc = ServoController()
