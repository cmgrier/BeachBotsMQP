#!/usr/bin/env python3
import math
import pyzed.sl as sl
import rospy
from baseBot.MapManager import MapManager
from support.Constants import *

# for testing
if __name__ == '__main__':
    mm = MapManager()
    timer = 0

    print("Getting Frames...")
    # Grab 500 frames and stop
    while timer < 500:
        mm.mapMaker.update_map_async()
        if mm.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1

    print(mm.get_visible_zones(mm.mapMaker.translation, mm.mapMaker.orientation))
    mm.update_OG()

    print(mm.occupancy_grid)
    rospy.loginfo("all tests passed")