#!/usr/bin/env python
#TODO update this test case

from support.Drive import Drive
from support.Constants import *
import sys
import rospy

if __name__  == "__main__":
    drive = Drive(SMALL_L_WHEEL_PIN, SMALL_R_WHEEL_PIN, SMALL_L_DIRECT_1, SMALL_L_DIRECT_2, SMALL_R_DIRECT_1, SMALL_R_DIRECT_2)

    if str(sys.argv[1]) == "speed":
        if(len(sys.argv) is 4):
            try:
                drive.run_wheels(sys.argv[2], sys.argv[3])
                rospy.sleep(10)
                drive.cleanup()
            
	    except KeyboardInterrupt:
                drive.cleanup()

    if str(sys.argv[1]) == "direction":
        try:
            drive.set_direction(str(sys.argv[2]), str(sys.argv[3]))
            drive.run_wheels(50, 50)
            rospy.sleep(10)
            drive.cleanup()

        except KeyboardInterrupt:
            drive.cleanup()
