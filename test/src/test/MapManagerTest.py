#!/usr/bin/env python
import math
import rospy
from baseBot.MapManager import MapManager
from support.Constants import *

# for testing
if __name__ == '__main__':
    mapManager = MapManager()
    map = [0] * 10000  # 100 by 100 height map of 0s
    map_width = int(math.sqrt(len(map)))
    mapManager.mapMaker.map = map
    mapManager.update_map()

    number_of_zones = int(map_width / ZONE_WIDTH)
    print(len(map))
    print(number_of_zones)

    assert len(mapManager.zones) == number_of_zones
    assert len(mapManager.landing_strip) == 4

    index_list = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    for i in range(0, 10):
        map[i] = -1

    assert mapManager.percent_of_indexes_safe(index_list) == 1.0
    rospy.loginfo("all tests passed")
